#include <stdio.h>
#include "pico/stdlib.h"

class PWMController 
{
    public:
        // Constructor
        PWMController(uint8_t pwmPin, uint32_t clockFrequency=125000000U, float clockDiv = 78.125f);

        // Initialization method
        void init(uint32_t clockFrequency, float clockDiv = 78.125f);  // 78.125f default clock div 

        // Method to update wrap value, period, and pulse width
        void updateTiming(uint32_t periodUs, uint32_t pulseWidthUs, uint32_t startUs);

        // Method to start PWM output
        void start(void);

        // Method to stop PWM output
        void stop(void);

        // Destructor
        ~PWMController();

        // Optional: Method to get the current clock divider (if needed)
        float getClockDiv(void) const { return _clockDiv; }

    private:
        uint8_t _pwmPin;
        uint _sliceNum;
        uint _channel;
        uint32_t _wrapValue;
        uint32_t _clockFrequency;
        float _clockDiv;  // Store the clock divider value
};

