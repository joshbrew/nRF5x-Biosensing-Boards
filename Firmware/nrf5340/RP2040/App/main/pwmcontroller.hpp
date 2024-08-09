#include <stdio.h>
#include "pico/stdlib.h"


class PWMController 
{
    public:
        PWMController(uint8_t pwmPin, uint32_t clockFrequency);
        void init(uint32_t clockFrequency);
        void updateWrapValue(uint32_t pulseWidthUs, uint32_t periodUs);
        void setPWMValues(void);
        void start(void);
        void stop(void);
        ~PWMController();

    private:
        uint8_t _pwmPin;
        uint _sliceNum;
        uint _channel;
        uint32_t _wrapValue;
        uint32_t _clockFrequency;
};
