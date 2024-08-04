#include <stdio.h>
#include "pico/stdlib.h"

#define ADC_SPS_A 250
#define ADC_SPS_B 500
#define ADC_SPS_C 1000
#define ADC_SPS_D 2000

#define PWM_LED_PIN_N1 2
#define PWM_LED_PIN_N2 4

#define INFRARED_LED_PIN PWM_LED_PIN_N1
#define RED_LED_PIN PWM_LED_PIN_N2

#define RPI_PICO_CLOCK_FREQUENCY 60000000U //Set to same clock used in main thread set_sys_clock_khz
#define PWM_CLOCK_DIVIDER 256.f
#define PWM_DUTY_CYCLE 0.25

class PWMController 
{
    public:
        PWMController(uint8_t pwmPin, uint8_t ledNumber);
        void init();
        void updateWrapValue(uint32_t pulseWidthUs, uint32_t periodUs);
        void setPWMValues(void);
        uint8_t getLedNumber(void);
        void start(void);
        void stop(void);
        ~PWMController();

    private:
        const uint8_t _ledNumber;
        uint8_t _pwmPin;
        uint _sliceNum;
        uint32_t _wrapValue;
};