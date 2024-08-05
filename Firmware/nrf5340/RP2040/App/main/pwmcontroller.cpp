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

/**
## updateWrapValue Calculation Steps

1. **PWM Timer Peripheral Clock Frequency Calculation:**
    - The PWM clock divider divides the Raspberry Pi clock frequency by 256, resulting in a PWM timer peripheral clock frequency of 488,281.25 Hz.

2. **Timer Tick Duration Calculation:**
    - With the PWM timer peripheral clock frequency, each timer tick duration is calculated as 2.048 microseconds (us).

3. **Target PWM Frequency:**
    - The desired PWM frequency is set to 250 Hz. We can select 500, 1000 or 2000.

4. **Counter Value Calculation:**
    - To achieve the desired PWM frequency, the total number of timer ticks required per cycle is calculated by dividing the PWM timer peripheral clock frequency (488,281.25 Hz) by the desired PWM frequency (250 Hz), resulting in approximately 1953.125 ticks.

5. **Total Cycle Duration Calculation:**
    - Multiplying the tick duration (2.048 us) by the total number of ticks (1953) gives the total cycle duration of the PWM signal, which is approximately 4 milliseconds (ms). This results in a PWM signal with a frequency of 250 Hz.

 */