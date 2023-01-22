#pragma once

#include <stddef.h>

/**
 * @brief Ids for commands sent via ble Gatt control characteristic
 */
enum class SensorId : uint8_t
{
    Ads131m08_0     = 2,
    Ads131m08_1     = 3,
    Mpu6050         = 4,
    Max30102        = 5,
    Bme280          = 6,        
};
