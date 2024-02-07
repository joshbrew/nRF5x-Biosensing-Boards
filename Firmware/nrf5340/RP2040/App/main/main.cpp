#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include <vector>
#include "pwmcontroller.hpp"
#include "uartcontroller.hpp"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

int main() 
{
    stdio_init_all();

    std::vector<PWMController> pwmControllers = {PWMController(RED_LED_PIN, 1), PWMController(INFRARED_LED_PIN, 2)};
    UARTController uartController(UART_ID, BAUD_RATE, TX_PIN, RX_PIN);

    std::string receivedString; 

    while (true) 
    {
        if (uartController.isReadable()) 
        {
            char receivedChar = uartController.read();

            if (receivedChar == '\n') 
            {
                // End of the string received, process it
                auto ledConfig = uartController.parse(receivedString);

                if (pwmControllers[0].getLedNumber() == ledConfig.first)
                {
                    pwmControllers[0].init();
                    pwmControllers[1].init();

                    pwmControllers[0].updateWrapValue(ledConfig.second);
                    pwmControllers[0].setPWMValues();
                    pwmControllers[1].updateWrapValue(ledConfig.second);
                    pwmControllers[1].setPWMValues();

                    pwmControllers[0].start();
                    busy_wait_us(pwmControllers[0].getPulseDelay());
                    pwmControllers[1].start();
                }
                // Clear the receivedString for the next input 
                receivedString.clear();
            } 
            else 
            {
                // Append the character to the received string
                receivedString += receivedChar;
            }
        }
    }

    return 0;
}
