#include "pwmcontroller.hpp"
#include "hardware/pwm.h"


PWMController::PWMController(uint8_t pwmPin, uint8_t ledNumber, uint32_t clockFrequency) 
    : _pwmPin(pwmPin), _ledNumber(ledNumber), _clockFrequency(clockFrequency) 
{
    // Initialize the PWM configuration
    init(clockFrequency);
}

void PWMController::init(uint32_t clockFrequency) 
{
    // Set the function of the GPIO pin to PWM
    gpio_set_function(_pwmPin, GPIO_FUNC_PWM);

    // Get the PWM slice number associated with the GPIO pin
    _sliceNum = pwm_gpio_to_slice_num(_pwmPin);

    // Get the PWM channel associated with the GPIO pin
    _channel = pwm_gpio_to_channel(_pwmPin);

    // Get the default PWM configuration
    pwm_config config = pwm_get_default_config();
    
    // Set the clock divider to 1 for custom timing (clock divider not used)
    pwm_config_set_clkdiv(&config, 1.0f);
    
    // Initialize the PWM slice with the configuration but do not start it yet
    pwm_init(_sliceNum, &config, false);
}

void PWMController::start() 
{
    // Enable the PWM signal on the specified slice
    pwm_set_enabled(_sliceNum, true);
}

void PWMController::stop() 
{
    // Disable the PWM signal on the specified slice
    pwm_set_enabled(_sliceNum, false);
}

// Update the PWM wrap value and calculate the duty cycle based on the configuration
void PWMController::updateWrapValue(uint32_t pulseWidthUs, uint32_t periodUs) 
{
    // Calculate the wrap value based on the period
    _wrapValue = (_clockFrequency / 1000000) * periodUs;
    
    // Calculate the duty cycle level based on the pulse width
    uint32_t level = (_wrapValue * pulseWidthUs) / periodUs;
    
    // Set the PWM wrap value to control the period
    pwm_set_wrap(_sliceNum, _wrapValue);
    
    // Set the PWM duty cycle to control the pulse width for the specific channel
    pwm_set_chan_level(_sliceNum, _channel, level);
}

// Get the LED number associated with this controller
uint8_t PWMController::getLedNumber(void)
{
    return _ledNumber;
}

// Destructor for PWMController
PWMController::~PWMController() {
    // Cleanup if needed (currently not used)
}

