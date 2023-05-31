#pragma once

#include <stddef.h>

/**
 * @brief The set of various BLE commands for different purposes
 */
enum class BleCommand : uint8_t
{
    StartSampling = 0x01,       ///< Start taking samples from the sensor
    StopSampling = 0x02,       ///< Stop taking samples from the sensor
};