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
#define DEFAULT_PULSE_WIDTH_US 250
#define DEFAULT_PERIOD_US (1000000 / 250) // 250Hz
#define DEFAULT_INITIAL_DELAY_US 0
#define CORE_CLOCK_KHZ 80000

#define BAUD_RATE 115200
#define TX_PIN 0
#define RX_PIN 1

#define WAKE_CHECK_INTERVAL 5 // Interval in seconds to wake up and check for a wake command


WS2812 ws2812(RGB_PIN, 1, PIO_INSTANCE, 0, WS2812::FORMAT_RGB);

void flicker() {
    ws2812.fill(WS2812::RGB(0,100,100)); // Teal
    ws2812.show();

    busy_wait_us(50000);
    
    ws2812.fill(WS2812::RGB(0,100,0)); // Green
    ws2812.show();

    busy_wait_us(50000);

    ws2812.fill(WS2812::RGB(0,0,0)); 
    ws2812.show();

}

void flickerB() {
    ws2812.fill(WS2812::RGB(0,0,100)); // Blue
    ws2812.show();

    busy_wait_us(100000);
    
    ws2812.fill(WS2812::RGB(100,0,100)); // Purple
    ws2812.show();

    busy_wait_us(100000);

    ws2812.fill(WS2812::RGB(0,0,100)); // Blue
    ws2812.show();

    busy_wait_us(100000);
    
    ws2812.fill(WS2812::RGB(100,0,100)); // Purple
    ws2812.show();

    busy_wait_us(100000);

    ws2812.fill(WS2812::RGB(0,0,0)); 
    ws2812.show();

}



// LED GPIO pins list with -1 representing NULL, so it will be used to subdivide the duty cycles
std::vector<int> gpioPins = { 5, -1, 8, -1 };//{ 5,  -1 };//{ 2, 3, 5, 6, 8, 9, -1 };
//IMPORTANT: SHARED PWM CHANNELS WILL BE FORCED TO USE THE SAME PHASE OFFSET, MEANING WE ONLY GET 8 PWMS NOT 16

std::vector<PWMController*> pwmControllers;

volatile uint32_t timingWindowUs = DEFAULT_PERIOD_US;
volatile uint32_t pulseWidthUs = DEFAULT_PULSE_WIDTH_US;
volatile uint32_t initialDelayUs = DEFAULT_INITIAL_DELAY_US;
volatile float defaultDutyCycle = 0.0f;
volatile bool running = false;
std::string receivedString; 
volatile size_t currentLed = 0;

// Function to select the period based on a preset character
uint32_t selectPerioduS(char preset) {
    switch (preset) {
        case 'A': return 4000; // 250Hz
        case 'B': return 2000; // 500Hz
        case 'C': return 1000; // 1000Hz
        case 'D': return 500;  // 2000Hz
        default: return DEFAULT_PERIOD_US; // Default to 250Hz
    }
}

void cleanupPWMControllers() {
    for (auto controller : pwmControllers) {
        if (controller != NULL) {
            controller->stop();
            delete controller;
        }
    }
    pwmControllers.clear();  // Clear the vector to remove all elements
}

void initPWMRoutine(std::vector<int> gpioPins) {
    cleanupPWMControllers();

    running = true;

    // Calculate the period for each controller by scaling the base period with the number of GPIO pins
    uint32_t totalPeriodUs = timingWindowUs * gpioPins.size();  // timingWindowUs is used as the base period, scaled by the number of pins

    size_t activeControllerIndex = 0;

    for (size_t i = 0; i < gpioPins.size(); i++) {
        int gpio = gpioPins[i];
        if (gpio != -1) {
            pwmControllers.push_back(new PWMController(gpio, DEFAULT_CLOCK_FREQUENCY));

            // Calculate the start time to evenly divide the period across all active controllers and blanks
            uint32_t startUs = (totalPeriodUs / gpioPins.size()) * i;

            // Update the PWM controller timing with the correct period, pulse width, and start time
            pwmControllers[activeControllerIndex]->updateTiming(totalPeriodUs, pulseWidthUs, startUs);


            activeControllerIndex++;
        }
    }

    activeControllerIndex = 0;
    for (size_t i = 0; i < gpioPins.size(); i++) {
        int gpio = gpioPins[i];
        if (gpio != -1) {
            // Start the PWM controller
            pwmControllers[activeControllerIndex]->start();
            activeControllerIndex++;
        }
    }
    flickerB(); // Confirm with LED blink
}

bool core1Launched = false;

// Core1 function to run PWM routine
void core1_entry() {
    // Initialize the PWM controllers on core1
    initPWMRoutine(gpioPins);

    // Core1 can now continuously manage PWM without interruptions from UART handling on core0
    // while (true) {
    //     // PWM updates or checks can be handled here, if necessary
    //     // For now, just let it run the initialized routine
    // }
}


// Function to stop core1 if it's already running
void stopCore1() {
    if (core1Launched) {
        multicore_reset_core1();
        core1Launched = false;
    }
}
// Function to start core1 with the PWM routine
void startCore1() {
    stopCore1(); // Ensure core1 is stopped before restarting
    multicore_launch_core1(core1_entry);
    core1Launched = true;
    printf("Core1 launched with PWM routine\n");
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



// Function to parse the received command
void parseCommand(const std::string& command) {

    printf("Received command: %s\n", command.c_str());

    if (command == "STOP") {
        running = false;
        stopCore1();
        for (auto& controller : pwmControllers) {
            if (controller != NULL) controller->stop();
        }
    } else if (command == "SLEEP") {
        stopCore1();
        for (auto& controller : pwmControllers) {
            if (controller != NULL) controller->stop();
        }
    } else if (command == "WAKE") {
        startCore1(); // Re-initialize the controllers and resume operation on core1
    } else if (command.length() == 1 && std::isalpha(command[0])) {
        // Handle frequency preset command
        timingWindowUs = selectPerioduS(command[0]);
        startCore1(); // Apply the new settings and re-launch core1

    } else {
        std::stringstream ss(command);
        std::string cmd;
        ss >> cmd;
        std::string parameter;

        uint32_t startUs = 0;
        uint32_t endUs = 0;
        uint32_t value;

        while (ss >> parameter >> value) {
            if (parameter == "PULSEWIDTH") {
                pulseWidthUs = value;
            } else if (parameter == "PERIOD") {
                timingWindowUs = value;
            } else if (parameter == "INITIALDELAY") {
                initialDelayUs = value;
            } else if (parameter == "STARTUS") {
                startUs = value;
            } else if (parameter == "ENDUS") {
                endUs = value;
            }
        }

        startCore1(); // Apply the new settings and re-launch core1 with updated timing
    }
}






int main() {
    stdio_init_all();

    printf("RP2040 Booted\n");

    // Set system clock to 80MHz
    set_sys_clock_khz(CORE_CLOCK_KHZ, true);
    

    // Initialize the UART controller
    UARTController uartController(UART_ID, BAUD_RATE, TX_PIN, RX_PIN);

    flicker();

    // Initialize the PWM controllers
    //parseCommand("A"); // Auto start to test

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
                
                //busy_wait_us(3000000);
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