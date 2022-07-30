#pragma once

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
//#include <drivers/i2c.h>
#include <sys/util.h>
#include <atomic>
#include "i2c_transport.hpp"
#include "device_string.hpp"

class UsbCommHandler;
//#define DT_DRV_COMPAT maxim_max30102

#define MAX30102_REG_INT_STS1 0x00
#define MAX30102_REG_INT_STS2 0x01
#define MAX30102_REG_INT_EN1 0x02
#define MAX30102_REG_INT_EN2 0x03
#define MAX30102_REG_FIFO_WR 0x04
#define MAX30102_REG_FIFO_OVF 0x05
#define MAX30102_REG_FIFO_RD 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CFG 0x08
#define MAX30102_REG_MODE_CFG 0x09
#define MAX30102_REG_SPO2_CFG 0x0a
#define MAX30102_REG_LED1_PA 0x0c
#define MAX30102_REG_LED2_PA 0x0d
/* #define MAX30102_REG_LED3_PA		0x0e */
#define MAX30102_REG_PILOT_PA 0x10
#define MAX30102_REG_MULTI_LED1 0x11
#define MAX30102_REG_MULTI_LED2 0x12
#define MAX30102_REG_TINT 0x1f
#define MAX30102_REG_TFRAC 0x20
#define MAX30102_REG_TEMP_CFG 0x21
#define MAX30102_REG_PROX_INT 0x30
#define MAX30102_REG_REV_ID 0xfe
#define MAX30102_REG_PART_ID 0xff

#define MAX30102_INT_PPG_MASK (1 << 6)

#define MAX30102_FIFO_CFG_SMP_AVE_SHIFT 5
#define MAX30102_FIFO_CFG_FIFO_FULL_SHIFT 0
#define MAX30102_FIFO_CFG_ROLLOVER_EN_MASK (1 << 4)

#define MAX30102_MODE_CFG_SHDN_MASK (1 << 7)
#define MAX30102_MODE_CFG_RESET_MASK (1 << 6)

#define MAX30102_SPO2_ADC_RGE_SHIFT 5
#define MAX30102_SPO2_SR_SHIFT 2
#define MAX30102_SPO2_PW_SHIFT 0

#define MAX30102_PART_ID 0x15

#define FIFO_A_FULL_MASK    (1 << 7)
#define PPG_RDY_MASK        (1 << 6)
#define ALC_OVF_MASK        (1 << 5)
#define PWR_RDY_MASK        (1 << 0)
#define DIE_TEMP_RDY_MASK   (1 << 1)

#define MAX30102_BYTES_PER_CHANNEL 3
#define MAX30102_MAX_NUM_CHANNELS 2
#define MAX30102_MAX_BYTES_PER_SAMPLE (MAX30102_MAX_NUM_CHANNELS * \
                                       MAX30102_BYTES_PER_CHANNEL)

#define MAX30102_SLOT_LED_MASK 0x03

#define MAX30102_FIFO_DATA_BITS 18
#define MAX30102_FIFO_DATA_MASK ((1 << MAX30102_FIFO_DATA_BITS) - 1)

#define MAX30102_INTR_2_DIE_TEMP_RDY_EN 2
#define MAX30102_TEMP_CFG_TEMP_EN 1

/* Interrupt Status 1 Register */
#define MAX30102_INTR_STATUS_1_A_FULL(x) (((x) >> 7) & 0x1)
#define MAX30102_INTR_STATUS_1_PPG_RDY(x) (((x) >> 6) & 0x1)
#define MAX30102_INTR_STATUS_1_ALC_OVF(x) (((x) >> 5) & 0x1)
#define MAX30102_INTR_STATUS_1_PWR_RDY(x) (((x) >> 0) & 0x1)

/* Interrupt Status 2 Register */
#define MAX30102_INTR_STATUS_2_DIE_TEMP_RDY(x) (((x) >> 1) & 0x1)

    struct max30102_config {
        uint8_t interrupt_config_1;
        uint8_t interrupt_config_2;
        uint8_t fifo_config;
        uint8_t mode_config;
        uint8_t spo2_config;
        uint8_t led_pa[MAX30102_MAX_NUM_CHANNELS];
        uint8_t slot_config[MAX30102_MAX_NUM_CHANNELS];
    };

    enum class max30102_mode : uint8_t {
        MAX30102_MODE_HEART_RATE = 2,
        MAX30102_MODE_SPO2 = 3,
        MAX30102_MODE_MULTI_LED = 7,
    };

    enum class max30102_slot : uint8_t {
        MAX30102_SLOT_DISABLED = 0,
        MAX30102_SLOT_RED_LED1_PA,
        MAX30102_SLOT_IR_LED2_PA,
        MAX30102_SLOT_GREEN_LED3_PA,
        MAX30102_SLOT_RED_PILOT_PA,
        MAX30102_SLOT_IR_PILOT_PA,
        MAX30102_SLOT_GREEN_PILOT_PA,
    };

    enum class max30102_led_channel : uint8_t {
        MAX30102_LED_CHANNEL_RED = 0,
        MAX30102_LED_CHANNEL_IR,
        MAX30102_LED_CHANNEL_GREEN,
    };

    enum class max30102_pw : uint8_t {
        MAX30102_PW_15BITS = 0,
        MAX30102_PW_16BITS,
        MAX30102_PW_17BITS,
        MAX30102_PW_18BITS,
    };

    enum class max30102_intr_1 : uint8_t {
        MAX30102_INTR_1_ALC_OVF_EN = 32,
        MAX30102_INTR_1_PPG_RDY_EN = 64,
        MAX30102_INTR_1_A_FULL_EN = 128,
    };

/**
 * @brief Max30102 driver
 */
class Max30102 {

    using I2C_1DeviceName = DeviceString<'I', '2', 'C', '_', '1'>; 

    constexpr static uint8_t max30102_i2c_address = 0x57; //I2C Address
    constexpr static uint8_t max30102_id = 0x15; // Part ID
    uint8_t tx_buf[194] = {};

    struct max30102_data {
        const struct device *i2c;
        uint32_t raw[MAX30102_MAX_NUM_CHANNELS];
        uint8_t map[MAX30102_MAX_NUM_CHANNELS];
        uint8_t num_channels;

        /* Die temperature */
        int8_t tint;
        uint8_t tfrac;
    };

public:
    /**
     * @brief Construct a new Max30102 object
     */
    Max30102(UsbCommHandler &controller);

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     * @return Status, 0 for no errors
     */
    int Initialize();    

    /**
     * @brief Configure Max30102 device
     * 
     * @param config Configuration details
     * @return Status, 0 for no errors
     */
    int Configure(max30102_config config);

    /**
     * @brief Start taking Red and IR samples
     */
    void StartSampling();    

    /**
     * @brief Stop taking Red and IR samples
     */
    void StopSampling();

    /**
     * @brief Initiate Single Temperature Reading. Max30102 will generate interrupt when temperature sample is available.
     */
    void InitiateTemperatureReading();    

    /**
     * @brief Handle Max30102 Interrupt. Figure out interrupt reason and take the appropriate action
     */
    void HandleInterrupt(); 

    /**
     * @brief Check if MAX30102 device is connected to I2C bus
     */
    bool IsOnI2cBus(); 

private:
    /**
     * @brief Reset Max30102 device
     */
    void Reset();

    /**
     * @brief Shutdown Max30102 device
     */
    void Shutdown();

    /**
     * @brief Wakeup Max30102 device
     */
    void Wakeup();
    
    /**
     * @brief Read Temperature Registers
     */
    void TemperatureRead();
    std::atomic<bool> max30102_is_on_i2c_bus_; ///< Device status
    I2CTransport<I2C_1DeviceName, max30102_i2c_address> transport; ///< I2C transport for device
    UsbCommHandler &serialHandler; ///< USB communication controller

};