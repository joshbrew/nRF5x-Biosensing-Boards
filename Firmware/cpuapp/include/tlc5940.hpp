#pragma once

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <atomic>
#include "ble_types.hpp"
#include "ble_commands.hpp"
              
struct tlc5940_config {
    struct gpio_dt_spec gsclk_gpio; /**< Reference clock for grayscale PWM control */
    struct gpio_dt_spec xlat_gpio; /**< Level triggered latch signal */
    struct gpio_dt_spec blank_gpio; /**< Blank all outputs */
    struct gpio_dt_spec sin_gpio; /**< Serial data input */
    struct gpio_dt_spec sclk_gpio; /**< Serial data shift clock */
};

enum pins: uint8_t {
    GSCLK_PIN = 0,
    XLAT_PIN,
    BLANK_PIN,
    SIN_PIN,
    SCLK_PIN
};

enum pulse_dir: uint8_t {
    LOW_TO_HIGH = 0,
    HIGH_TO_LOW
};

/**
 * @brief Tlc5940 driver
 */
class Tlc5940 {

public:
    /**
     * @brief Construct a new Tlc5940 object
     */
    Tlc5940();

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     * 
     * @param num_tlcs Number of daisy-chained TLC5940 devices
     * @return Status, 0 for no errors
     */
    int Initialize(uint8_t num_tlcs);

    /**
     * @brief Sets the grayscale data for channel
     * @note Tlc5940.Update function needs to be called after Set
     * 
     * @param channel Tlc5940 channel ID. 0 - (NUM_TLCS * 16 - 1)
     *                OUT0 of the first TLC is channel 0, OUT0 of the next TLC is channel 16, etc.
     * @param value Grayscale value to set. 0 - 4095.
     */
    void Set(uint8_t channel, uint16_t value);

    /**
     * @brief Set all channels to specified grayscale value
     * @note Tlc5940.Update function needs to be called after SetAll
     * 
     * @param value Grayscale value to set. 0 - 4095.
     */
    void SetAll(uint16_t value);

    /**
     * @brief Clears the GrenScale data array
     * 
     * @note Tlc5940.Update function needs to be called after Clear
     */
    void Clear();

    /**
     * @brief Get the current GrayScale value for a channel
     * 
     * @param channel Tlc5940 channel ID. 0 - (NUM_TLCS * 16 - 1)
     *                OUT0 of the first TLC is channel 0, OUT0 of the next TLC is channel 16, etc.
     * @return current GrayScale value for specified channel. 0 - 4095.
     */
    uint16_t Get(uint8_t channel);

    /**
     * @brief Shifts in the data from the GrayScale data array into TLC5940
     * @note If data has already been shifted in this GrayScale cycle, another call to Update() will immediatelly return -1
     *       without shifting in the new data. To ensure that a call to Update() does shift in ne data, use 
     *       while(Tlc5940.Update())
     * 
     * @return 0 if data was successfully shifted, -1 if there is data waiting to be latched
     */
    int Update();

private:

    int InitializeGpios(void);
    int SetPin(uint8_t pin, uint8_t state);
    int PulsePin(uint8_t pin, uint8_t pulse_dir);

    /**
     * @brief Called when trigger mode is received via BLE
     * 
     * @param buffer receviced buffer
     * @param length buffer length
     * @param offset data offset
     * 
     * @return true if command was processed succesfully
     */
    bool OnBleCommand(const uint8_t* buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset);

    struct tlc5940_config tlc_cfg;
    uint8_t num_tlcs; //< Number of daisy-chained TLC5940 devices
    const struct device *gpio0;
};

