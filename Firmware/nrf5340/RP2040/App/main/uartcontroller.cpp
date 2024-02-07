#include "uartcontroller.hpp"
#include <vector>

UARTController::UARTController(uart_inst_t* uartId, uint baudRate, uint txPin, uint rxPin) 
:   _uartId(uartId),
    _baudRate(baudRate) 
{
    init(txPin, rxPin);
}

void UARTController::init(uint txPin, uint rxPin) 
{
    // Set up UART
    uart_init(_uartId, _baudRate);
    gpio_set_function(txPin, GPIO_FUNC_UART); // Assuming UART TX is on GPIO 0
    gpio_set_function(rxPin, GPIO_FUNC_UART); // Assuming UART RX is on GPIO 1
}

bool UARTController::isReadable() 
{
    return uart_is_readable(_uartId);
}

char UARTController::read() 
{
    return uart_getc(_uartId);
}

void UARTController::write(char c) 
{
    uart_putc(_uartId, c);
}

std::pair<uint32_t, char> UARTController::parse(const std::string &input) 
{
    size_t ledNumPos = input.find("ledNum=");
    size_t configPos = input.find("Config=");

    if (ledNumPos != std::string::npos && configPos != std::string::npos) 
    {
        size_t commaPos = input.find(",", ledNumPos);
        if (commaPos != std::string::npos) 
        {
            std::string ledNumStr = input.substr(ledNumPos + 7, commaPos - (ledNumPos + 7));
            uint32_t ledNum = std::stoi(ledNumStr);
            printf("LED NUMBER = %d\r\n", ledNum);

            char configChar = input[configPos + 7];
            printf("Config = %c\r\n", configChar);

            return std::make_pair(ledNum, configChar);
        }
    }

    // Default value if parsing fails
    return std::make_pair(0, ' ');
}

UARTController::~UARTController() 
{
    // Cleanup if needed
}
