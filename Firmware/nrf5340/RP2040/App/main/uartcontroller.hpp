#include <stdio.h>
#include "pico/stdlib.h"
#include <utility>
#include <string>

#define UART_ID uart0
#define BAUD_RATE 115200
#define TX_PIN 0
#define RX_PIN 1

class UARTController 
{
    public:
        UARTController(uart_inst_t* uartId, uint baudRate, uint txPin, uint rxPin);
        void init(uint txPin, uint rxPin);
        bool isReadable();
        char read();
        void write(char c);
        std::pair<uint32_t, char> parse(const std::string &input);
        ~UARTController();

    private:
        uart_inst_t* _uartId;
        uint _baudRate;
};