#pragma once
#include <atomic>
#include <drivers/i2c.h>

/**
 * @brief template transport class used to read/write device registers via i2c bus
 * 
 * @tparam DeviceName Device name 
 * @tparam i2c_id  Device ID
 */
template <class DeviceName, int i2c_id>
class I2CTransport
{
public:
    /**
     * @brief Constructor
     */
    I2CTransport()
    {
        dev = device_get_binding(DeviceName::c_str());
        deviceStatus.store(0, std::memory_order_relaxed);
    }

    /**
     * @brief Initialization function used to intialize when underlayer infrastructure is ready
     */
    void Initialize()
    {
        int status = i2c_configure(dev, I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_MASTER);
        deviceStatus.store(status, std::memory_order_relaxed);
    }

    /**
     * @brief Write device Register
     * 
     * @param registerId Id of the register to write
     * @param value value to write
     */
    void WriteRegister(uint8_t registerId, uint8_t value)
    {
        int status = i2c_reg_write_byte(dev, i2c_id, registerId, value);
        deviceStatus.store(status, std::memory_order_relaxed);
    }

    /**
     * @brief Read device register
     * 
     * @param registerId Id of the register to read
     * @return uint8_t hardware register id
     */
    uint8_t ReadRegister(uint8_t registerId)
    {
        uint8_t value;
        int status = i2c_reg_read_byte(dev, i2c_id, registerId, &value);
        deviceStatus.store(status, std::memory_order_relaxed);
        return value;
    }

    /**
     * @brief Update hardware register. Reads hardware register updated register bits selected with mask
     *        and writes updated register value back
     * 
     * @param registerId Register ID to update
     * @param mask Bitmask of the register to update
     * @param value value for register
     */
    void UpdateRegister(uint8_t registerId, uint8_t mask, uint8_t value)
    {
        int status = i2c_reg_update_byte(dev, i2c_id, registerId, mask, value);
        deviceStatus.store(status, std::memory_order_relaxed);
    }

    /**
     * @brief Read hardware register batch from device
     * 
     * @param registerId Start register ID 
     * @param data pointer to data object, where register batch should be stored
     * @param size number of registers to read
     */
    void ReadRegisters(uint8_t registerId, uint8_t* data, int size)
    {
        int status = i2c_burst_read(dev, i2c_id, registerId, data, size);
        deviceStatus.store(status, std::memory_order_relaxed);
    }

    /**
     * @brief Get device status
     * 
     * @return Current status, 0 for no errors
     */
    uint8_t GetStatus()
    {
        int status = deviceStatus.load(std::memory_order_relaxed);

        return static_cast<uint8_t>(-status);
    }

private:
    std::atomic<int> deviceStatus; ///< Device status
    const device* dev = nullptr; ///< Logical device
};