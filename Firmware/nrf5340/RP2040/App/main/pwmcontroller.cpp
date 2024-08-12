#include "pwmcontroller.hpp"
#include "hardware/pwm.h"

// Constructor
PWMController::PWMController(uint8_t pwmPin, uint32_t clockFrequency, float clockDiv)
    : _pwmPin(pwmPin), _clockFrequency(clockFrequency), _clockDiv(clockDiv)
{
    // Initialize the PWM configuration
    init(_clockFrequency, _clockDiv);
}

// Initialization method
void PWMController::init(uint32_t clockFrequency, float clockDiv) 
{
    // Store the clock divider as a class variable
    _clockDiv = clockDiv;

    // Set the function of the GPIO pin to PWM
    gpio_set_function(_pwmPin, GPIO_FUNC_PWM);

    // Get the PWM slice number associated with the GPIO pin
    _sliceNum = pwm_gpio_to_slice_num(_pwmPin);

    // Get the PWM channel associated with the GPIO pin
    _channel = pwm_gpio_to_channel(_pwmPin);

    // Get the default PWM configuration
    pwm_config config = pwm_get_default_config();
    
    // Set the clock divider as needed
    pwm_config_set_clkdiv(&config, _clockDiv);
    
    // Initialize the PWM slice with the configuration but do not start it yet
    pwm_init(_sliceNum, &config, false);
}

void PWMController::updateTiming(uint32_t periodUs, uint32_t pulseWidthUs, uint32_t startUs) 
{
    // Calculate the wrap value based on the system clock frequency, clock divider, and the desired period
    _wrapValue = (uint16_t)((_clockFrequency / (_clockDiv * 1000000.0f)) * periodUs);

    // Calculate the level based on the pulse width (duration for which the signal is high) in clock cycles
    uint16_t level = (uint16_t)((_wrapValue * pulseWidthUs) / periodUs);

    // Enable phase-correct mode on this PWM slice
    //pwm_set_phase_correct(_sliceNum, true);

    // Set the PWM wrap value to control the period
    pwm_set_wrap(_sliceNum, _wrapValue);

    // Set the PWM duty cycle based on the level calculated
    pwm_set_chan_level(_sliceNum, _channel, level);

    // Optionally adjust the counter to start at a specific phase within the wrap range, if needed
    if (startUs > 0) {
        uint16_t startCount = (uint16_t)((_wrapValue * startUs) / periodUs);
        if (startCount >= _wrapValue) startCount = _wrapValue - 1;  // Cap startCount to prevent overflow
        pwm_set_counter(_sliceNum, startCount);
    }

    // Debugging information
    // printf("Pin: %d, Wrap: %d, Level: %d, Pulse Width: %d us, Start Count: %d, Clock Div: %.2f\n", 
    //        _pwmPin, _wrapValue, level, pulseWidthUs, startCount, _clockDiv);
}

// Start PWM output
void PWMController::start() 
{
    pwm_set_enabled(_sliceNum, true);
}

// Stop PWM output
void PWMController::stop() 
{
    pwm_set_enabled(_sliceNum, false);
}

// Destructor
PWMController::~PWMController() 
{
    // Disable the PWM signal on the specified slice
    pwm_set_enabled(_sliceNum, false);

    // Optionally, reset the GPIO function to its default state if needed
    gpio_set_function(_pwmPin, GPIO_FUNC_SIO);  // Set to SIO (default state)
}
