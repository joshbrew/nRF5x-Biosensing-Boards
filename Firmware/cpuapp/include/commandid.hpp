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
    DmicCmd = 7,   ///< Commands for Digital Microphone (DMIC)
    Tlc5940Cmd = 8, /// < COmmand for Tlc5940 16-channel PWM LED driver 
    BleCmd = 9,    ///< Command for start/stop looking for iBeacon devices
    SystemCmd = 10, ///< For sending commands on the app/system level from where we can control every sensor/module
};
