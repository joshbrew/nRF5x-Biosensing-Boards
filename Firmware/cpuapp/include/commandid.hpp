#pragma once

#include <stddef.h>

/**
 * @brief Ids for commands sent via ble Gatt control characteristic
 */
enum class CommandId : uint8_t
{
    Ads131m08Cmd = 1,
    Bme280Cmd = 2,
    Max30102Cmd = 3,
    Mpu6050Cmd = 4,
    Qmc5883lCmd = 5,
    AlarmCmd = 6,  ///< Start/Stop playing Alarm
};
