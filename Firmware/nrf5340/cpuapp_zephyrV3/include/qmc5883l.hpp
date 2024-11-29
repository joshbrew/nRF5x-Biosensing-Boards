#pragma once

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
//#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include "i2c_transport.hpp"
#include "device_string.hpp"
#include "ble_types.hpp"
#include "ble_commands.hpp"

class UsbCommHandler;

#define QMC5883L_ADDRESS            0x0D

// Internal I2C register list
#define QMC5883L_X_LSB              0x00
#define QMC5883L_X_MSB              0x01
#define QMC5883L_Y_LSB              0x02
#define QMC5883L_Y_MSB              0x03
#define QMC5883L_Z_LSB              0x04
#define QMC5883L_Z_MSB              0x05
#define QMC5883L_STATUS_REG         0x06
#define QMC5883L_TOUT_LSB           0x07
#define QMC5883L_TOUT_MSB           0x08
#define QMC5883L_CTRL_REG_1         0x09
#define QMC5883L_CTRL_REG_2         0x0A
#define QMC5883L_SET_RESET_PERIOD   0x0B
#define QMC5883L_WHO_AM_I           0x0D

#define QMC5883L_DRDY_BIT           0
#define QMC5883L_OVL_BIT            1
#define QMC5883L_DOR_BIT            2

#define QMC5883L_SOFT_RST_BIT       7

#define QMC5833L_MODE_MASK          (3 << 0)
#define QMC5833L_ODR_MASK           (3 << 2)
#define QMC5833L_RNG_MASK           (3 << 4)
#define QMC5833L_OSR_MASK           (3 << 6)

// Mode
#define QMC5833L_MODE_STANDBY       (0x00)
#define QMC5833L_MODE_CONTINUOUS    (0x01)
// Output Data Rate (ODR)
#define QMC5833L_ODR_10Hz           (0x00)
#define QMC5833L_ODR_50Hz           (0x01)
#define QMC5833L_ODR_100Hz          (0x02)
#define QMC5833L_ODR_200Hz          (0x03)
// Full scale (FS)
#define QMC5833L_FS_2G              (0x00)
#define QMC5833L_FS_8G              (0x01)
// Over Sample Ratio (OSR)
#define QMC5833L_OSR_512            (0x00)
#define QMC5833L_OSR_256            (0x01)
#define QMC5833L_OSR_128            (0x02)
#define QMC5833L_OSR_64             (0x03)

struct qmc5883l_config {
    uint8_t ctrl_reg_1;
    uint8_t ctrl_reg_2;
};

/**
 * @brief Qmc5883l driver
 */
class Qmc5883l {

    // using I2C_1DeviceName = DeviceString<'I', '2', 'C', '_', '1'>;
    using I2C_1DeviceName = DeviceString<'i', '2', 'c', '1'>;

    constexpr static uint8_t qmc5883l_id = 0xFF; // Part ID
    uint8_t tx_buf[250] = {};

public:
    /**
     * @brief Construct a new Qmc5883l object
     */
    Qmc5883l(UsbCommHandler &controller);

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     * @return Status, 0 for no errors
     */
    int Initialize();

    /**
     * @brief Configure Qmc5883l device
     *
     * @param config Configuration details
     * @return Status, 0 for no errors
     */
    int Configure(qmc5883l_config config);

    /**
     * @brief Start taking Accel and Gyro samples
     */
    void StartSampling();

    /**
     * @brief Stop taking Accel and Gyro samples
     */
    void StopSampling();

    /**
     * @brief Get Qmc5883l Die Temperature.
     */
    void GetDieTemperature();

    /**
     * @brief Handle Qmc5883l Interrupt. Figure out interrupt reason and take the appropriate action
     */
    void HandleInterrupt();

    /**
     * @brief Check if Qmc5883l device is connected to I2C bus
     */
    bool IsOnI2cBus();

private:
    /**
     * @brief Reset Qmc5883l device
     */
    void Reset();

    /**
     * @brief Shutdown Qmc5883l device
     */
    void Shutdown();

    /**
     * @brief Wakeup Qmc5883l device
     */
    void Wakeup();

    /**
     * @brief Read Temperature Registers
     */
    void TemperatureRead();

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


    uint8_t sample_cnt;
    uint8_t packet_cnt;
    std::atomic<bool> qmc5883l_is_on_i2c_bus_; ///< Device status
    I2CTransport<I2C_1DeviceName, QMC5883L_ADDRESS> transport; ///< I2C transport for device
    UsbCommHandler &serialHandler; ///< USB communication controller

};

