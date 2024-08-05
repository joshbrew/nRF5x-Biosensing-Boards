#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include <vector>
#include <sstream>
#include "pwmcontroller.hpp"
#include "uartcontroller.hpp"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"
//#include "pico/sleep.h"
#include "pico/multicore.h"

#define ADC_SPS_A 250
#define ADC_SPS_B 500
#define ADC_SPS_C 1000
#define ADC_SPS_D 2000

#define DEFAULT_CLOCK_FREQUENCY 80000000U // 80 MHz clock frequency
#define DEFAULT_PULSE_WIDTH_US 200
#define DEFAULT_PERIOD_US (1000000 / 250) // 250Hz
#define DEFAULT_INITIAL_DELAY_US 0
#define CORE_CLOCK_KHZ 80000

#define WAKE_CHECK_INTERVAL 5 // Interval in seconds to wake up and check for a wake command

// Function to select the period based on a preset character
uint32_t selectPeriod(char preset) {
    switch (preset) {
        case 'A': return 1000000 / 250; // 250Hz
        case 'B': return 1000000 / 500; // 500Hz
        case 'C': return 1000000 / 1000; // 1000Hz
        case 'D': return 1000000 / 2000; // 2000Hz
        default: return DEFAULT_PERIOD_US; // Default to 250Hz
    }
}

// Function to enter deep sleep and periodically wake up to check for a wake command
// void enterPeriodicDeepSleep() {
//     while (true) {
//         // Set RTC to wake up the microcontroller after WAKE_CHECK_INTERVAL seconds
//         datetime_t t;
//         rtc_get_datetime(&t);
//         t.sec += WAKE_CHECK_INTERVAL;
//         if (t.sec >= 60) {
//             t.sec -= 60;
//             t.min += 1;
//         }
//         if (t.min >= 60) {
//             t.min -= 60;
//             t.hour += 1;
//         }
//         if (t.hour >= 24) {
//             t.hour -= 24;
//             t.dotw += 1;
//         }
//         rtc_set_alarm(&t, NULL);

//         // Enter sleep
//         sleep_run_from_rtc();

//         // Check for a wake command
//         std::string receivedString;
//         UARTController uartController(UART_ID, BAUD_RATE, TX_PIN, RX_PIN);
//         while (uartController.isReadable()) {
//             char receivedChar = uartController.read();
//             if (receivedChar == '\n') {
//                 if (receivedString == "WAKE") {
//                     return; // Exit sleep loop if WAKE command is received
//                 }
//                 receivedString.clear();
//             } else {
//                 receivedString += receivedChar;
//             }
//         }
//     }
// }


// Global variables to share data between cores
std::vector<PWMController> pwmControllers = {
    PWMController(2, 1, DEFAULT_CLOCK_FREQUENCY),
    PWMController(3, 2, DEFAULT_CLOCK_FREQUENCY),
    PWMController(5, 4, DEFAULT_CLOCK_FREQUENCY),
    PWMController(6, 5, DEFAULT_CLOCK_FREQUENCY),
    PWMController(8, 7, DEFAULT_CLOCK_FREQUENCY),
    PWMController(9, 8, DEFAULT_CLOCK_FREQUENCY)
};
volatile uint32_t periodUs = DEFAULT_PERIOD_US;
volatile uint32_t pulseWidthUs = DEFAULT_PULSE_WIDTH_US;
volatile uint32_t initialDelayUs = DEFAULT_INITIAL_DELAY_US;
volatile bool running = true;
std::string receivedString; 

// Core 1 entry function
void core1_entry() {
    size_t currentLed = 0;

    while (running) {
        // Apply initial delay before starting the first pulse
        if (initialDelayUs > 0) busy_wait_us(initialDelayUs);

        // Start the current LED
        pwmControllers[currentLed].start();
        // Wait for the pulse width
        busy_wait_us(pulseWidthUs);
        // Stop the current LED
        pwmControllers[currentLed].stop();
        // Wait for the remainder of the period
        if (periodUs > 0) busy_wait_us(periodUs - pulseWidthUs);

        // Move to the next LED
        currentLed = (currentLed + 1) % pwmControllers.size();
    }
}

bool launched = false;

void initPWMRoutine() {
    running = true;
    for (auto& controller : pwmControllers) {
        controller.init(DEFAULT_CLOCK_FREQUENCY);
        controller.updateWrapValue(pulseWidthUs, periodUs); // Use current pulse width and period
    }
    if(!launched) {
        multicore_launch_core1(core1_entry); // Restart core 1 if needed
        launched = true;
    }
}

// Function to parse the received command
void parseCommand(const std::string& command) {
    std::stringstream ss(command);
    std::string cmd;
    ss >> cmd;

    if (cmd == "STOP") {
        running = false;
        for (auto& controller : pwmControllers) {
            controller.stop();
        }
    } else if (cmd == "SLEEP") {
        // Enter periodic deep sleep
        for (auto& controller : pwmControllers) {
            controller.stop();
        }
    } else if (cmd == "WAKE") {
        // Re-initialize the controllers and resume operation
        initPWMRoutine();
    } else if (cmd.length() == 1 && std::isalpha(cmd[0])) {
        // Handle frequency preset command
        periodUs = selectPeriod(cmd[0]);
        // Apply the new settings
        for (auto& controller : pwmControllers) {
            controller.updateWrapValue(pulseWidthUs, periodUs);
        }

    } else {
        // Assume the command is to set period, pulse width, or initial delay
        std::string parameter;
        uint32_t value;
        while (ss >> parameter >> value) {
            if (parameter == "PULSEWIDTH") {
                pulseWidthUs = value;
            } else if (parameter == "PERIOD") {
                periodUs = value;
            } else if (parameter == "INITIALDELAY") {
                initialDelayUs = value;
            }
        }

        // Apply the new settings
        for (auto& controller : pwmControllers) {
            controller.updateWrapValue(pulseWidthUs, periodUs);
        }
    }
}

int main() {
    stdio_init_all();

    // Set system clock to 80MHz
    set_sys_clock_khz(CORE_CLOCK_KHZ, true);
    
    // Initialize the UART controller
    UARTController uartController(UART_ID, BAUD_RATE, TX_PIN, RX_PIN);

    // Initialize the PWM controllers
    initPWMRoutine();
    parseCommand("A"); // Auto start to test

    while (true) {
        if (uartController.isReadable()) {
            char receivedChar = uartController.read();

            if (receivedChar == '\r') {
                // Ignore carriage return
                continue;
            } else if (receivedChar == '\n') {
                // End of the string received, process it
                parseCommand(receivedString);

                // Clear the receivedString for the next input 
                receivedString.clear();
            } else {
                // Append the character to the received string
                receivedString += receivedChar;
            }
        }
    }

    return 0;
}