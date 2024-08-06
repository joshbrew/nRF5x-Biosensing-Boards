#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include <vector>
#include <sstream>

#include "pwmcontroller.hpp"
#include "uartcontroller.hpp"
#include "WS2812.hpp"

#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"
//#include "pico/sleep.h"
#include "pico/multicore.h"

#define RGB_PIN 16
#define PIO_INSTANCE pio1


#define ADC_SPS_A 250
#define ADC_SPS_B 500
#define ADC_SPS_C 1000
#define ADC_SPS_D 2000

#define DEFAULT_CLOCK_FREQUENCY 80000000U // 80 MHz clock frequency
#define DEFAULT_PULSE_WIDTH_US 500
#define DEFAULT_PERIOD_US (1000000 / 250) // 250Hz
#define DEFAULT_INITIAL_DELAY_US 0
#define CORE_CLOCK_KHZ 80000

#define BAUD_RATE 115200
#define TX_PIN 0
#define RX_PIN 1

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
    PWMController(2, 1, DEFAULT_CLOCK_FREQUENCY), //r
    PWMController(3, 2, DEFAULT_CLOCK_FREQUENCY), //ir
    //PWMController(4, 3, DEFAULT_CLOCK_FREQUENCY),
    PWMController(5, 4, DEFAULT_CLOCK_FREQUENCY), //r
    PWMController(6, 5, DEFAULT_CLOCK_FREQUENCY), //ir
    // //PWMController(7, 6, DEFAULT_CLOCK_FREQUENCY),
    PWMController(8, 7, DEFAULT_CLOCK_FREQUENCY), //r
    PWMController(9, 8, DEFAULT_CLOCK_FREQUENCY)  //ir
    //PWMController(10, 9, DEFAULT_CLOCK_FREQUENCY)
};

volatile uint32_t periodUs = DEFAULT_PERIOD_US;
volatile uint32_t pulseWidthUs = DEFAULT_PULSE_WIDTH_US;
volatile uint32_t initialDelayUs = DEFAULT_INITIAL_DELAY_US;
volatile bool running = true;
std::string receivedString; 
volatile size_t currentLed = 0;

bool repeating_timer_callback(repeating_timer_t  *t) {
    if (!running) return false;  // Stop the timer if the running flag is false

    if (initialDelayUs > 0) busy_wait_us(initialDelayUs);

    pwmControllers[currentLed].start();
    busy_wait_us(pulseWidthUs);
    pwmControllers[currentLed].stop();

    currentLed = (currentLed + 1) % pwmControllers.size();
    return true;  // Keep repeating the timer
}

void core1_entry() {
    repeating_timer_t timer;

    // Set up the initial delay if required
    WS2812 ws2812(RGB_PIN, 1, PIO_INSTANCE, 0, WS2812::FORMAT_RGB);

    // Initialize the repeating timer
    bool timer_added = add_repeating_timer_us(-static_cast<int64_t>(periodUs), repeating_timer_callback, NULL, &timer);

    if (timer_added) {
        // printf("Timer added successfully\n");
        // ws2812.fill(WS2812::RGB(0, 255, 0)); // Green
        // ws2812.show();
    } else {
        //printf("Failed to add timer\n");
        ws2812.fill(WS2812::RGB(255, 0, 0)); // Red
        ws2812.show();
        return;
    }

    // Main loop can remain to check if the running flag changes and stop the timer
    while (true) {
        //tight_loop_contents();  // Low-power wait
    }

    // If running is set to false, remove the timer
    cancel_repeating_timer(&timer);
}


bool launched = false;

void initPWMRoutine() {
    running = true;
    for (auto& controller : pwmControllers) {
        controller.updateWrapValue(pulseWidthUs, periodUs); // Use current pulse width and period
    }
    if(!launched) {
        multicore_launch_core1(core1_entry); // Restart core 1 if needed
        launched = true;
    }
}

// Function to parse the received command
void parseCommand(const std::string& command) {

    printf("Received command: %s\n", command.c_str());

    if (command == "STOP") {
        running = false;
        for (auto& controller : pwmControllers) {
            controller.stop();
        }
    } else if (command == "SLEEP") {
        // Enter periodic deep sleep
        for (auto& controller : pwmControllers) {
            controller.stop();
        }
    } else if (command == "WAKE") {
        // Re-initialize the controllers and resume operation
        initPWMRoutine();
    } else if (command.length() == 1 && std::isalpha(command[0])) {
        // Handle frequency preset command
        periodUs = selectPeriod(command[0]);
        // Apply the new settings
        initPWMRoutine();

    } else {
        std::stringstream ss(command);
        std::string cmd;
        ss >> cmd;
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

    printf("RP2040 Booted\n");

    // Set system clock to 80MHz
    set_sys_clock_khz(CORE_CLOCK_KHZ, true);
    
    WS2812 ws2812(RGB_PIN, 1, PIO_INSTANCE, 0, WS2812::FORMAT_RGB);

    // Initialize the UART controller
    UARTController uartController(UART_ID, BAUD_RATE, TX_PIN, RX_PIN);

    ws2812.fill(WS2812::RGB(155,0,255)); // Purple
    ws2812.show();

    busy_wait_us(100000);

    ws2812.fill(WS2812::RGB(0,0,0)); // Purple
    ws2812.show();

    for (auto& controller : pwmControllers) {
        controller.init(DEFAULT_CLOCK_FREQUENCY);
    }
    // Initialize the PWM controllers
    parseCommand("A"); // Auto start to test

    printf("RP2040 Running\n");


    while (true) {
        if (uartController.isReadable()) {
            char receivedChar = uartController.read();

            //printf("Char: %c (ASCII: %d)\n", receivedChar, (int)receivedChar);

            if (receivedChar == '\r') {
                // Ignore carriage return
                continue;
            } else if (receivedChar == '\n') {
                // Debug print to show the accumulated command before parsing
                //printf("Complete command: %s\n", receivedString.c_str());
                
                ws2812.fill(WS2812::RGB(0,255,0)); // Green
                ws2812.show();

                // End of the string received, process it
                parseCommand(receivedString);

                // Clear the receivedString for the next input 
                receivedString.clear();
                
                ws2812.fill(WS2812::RGB(0,0,0)); // 0ff
                ws2812.show();
            } else if ((int)receivedChar > 0) {
                // Append the character to the received string
                receivedString += receivedChar;
                // Debug print to show the string length and its current content
                //printf("Received string so far: %s (Length: %d)\n", receivedString.c_str(), receivedString.length());
            }
        }

    }

    return 0;
}