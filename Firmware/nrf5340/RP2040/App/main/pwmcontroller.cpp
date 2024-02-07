#include "pwmcontroller.hpp"
#include "hardware/pwm.h"

PWMController::PWMController(uint8_t pwmPin, uint8_t ledNumber) : _pwmPin(pwmPin), _ledNumber(ledNumber) 
{
    init();
}

void PWMController::init() 
{
    gpio_set_function(_pwmPin, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the PWM pin
    _sliceNum = pwm_gpio_to_slice_num(_pwmPin);

    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, PWM_CLOCK_DIVIDER);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(_sliceNum, &config, false);
}

void PWMController::start() 
{
    pwm_set_enabled(_sliceNum, true);
}

void PWMController::stop() 
{
    pwm_set_enabled(_sliceNum, false);
}

void PWMController::updateWrapValue(char configChar) 
{
    uint32_t onTimeUs = 0;
    uint32_t risingEdgeDuration = 0;

    switch (configChar) 
    {
        case 'A':
            _wrapValue = (RPI_PICO_CLOCK_FREQUENCY / PWM_CLOCK_DIVIDER) / ADC_SPS_A;
            onTimeUs = ((1.f / ADC_SPS_A) * PWM_DUTY_CYCLE) * 1000000;
            risingEdgeDuration = DELAY_BETWEEN_PULSES_US * 4;
            _pulseDelay = onTimeUs + risingEdgeDuration;
            break;
        case 'B':
            _wrapValue = (RPI_PICO_CLOCK_FREQUENCY / PWM_CLOCK_DIVIDER) / ADC_SPS_B;
            onTimeUs = ((1.f / ADC_SPS_B) * PWM_DUTY_CYCLE) * 1000000;
            risingEdgeDuration = DELAY_BETWEEN_PULSES_US * 3;
            _pulseDelay = onTimeUs + risingEdgeDuration;
            break;
        case 'C':
            _wrapValue = (RPI_PICO_CLOCK_FREQUENCY / PWM_CLOCK_DIVIDER) / ADC_SPS_C;
            onTimeUs = ((1.f / ADC_SPS_C) * PWM_DUTY_CYCLE) * 1000000;
            risingEdgeDuration = DELAY_BETWEEN_PULSES_US * 2;
            _pulseDelay = onTimeUs + risingEdgeDuration;
            break;
        case 'D':
            _wrapValue = (RPI_PICO_CLOCK_FREQUENCY / PWM_CLOCK_DIVIDER) / ADC_SPS_D;
            onTimeUs = ((1.f / ADC_SPS_D) * PWM_DUTY_CYCLE) * 1000000;
            risingEdgeDuration = DELAY_BETWEEN_PULSES_US;
            _pulseDelay = onTimeUs + risingEdgeDuration;
            break;
        default:
            // No match found
            break;
    }

}

void PWMController::setPWMValues(void) 
{
    pwm_set_wrap(_sliceNum, _wrapValue);
    pwm_set_gpio_level(_pwmPin, _wrapValue * PWM_DUTY_CYCLE);
}

uint8_t PWMController::getLedNumber(void)
{
    return _ledNumber;
}

uint64_t PWMController::getPulseDelay(void)
{
    return _pulseDelay;
}

PWMController::~PWMController() {
    // Cleanup if needed
}