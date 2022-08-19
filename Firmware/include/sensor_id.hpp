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
#if 0    
    ConfigureLeds = 2,            ///< Configure Leds command
    NightMode = 3,                ///< Enable/Disable night mode command
    PlayHaptic = 4,               ///< Play haptic on selected haptic motors
    StopHaptic = 5,               ///< Stops playing haptic
    SetSpecialTriggerMode = 6,    ///< Sets special trigger mode
    GetControllersInput = 6,      ///< Get input data for all controllers connected to stm8
    SetSensorCalibrationData = 7, ///< Sets sensor calibration data
    GetSensorCalibrationData = 8, ///< Gets sensor calibration data
    GetFwVersion = 9,             ///< Gets firmware version
    GetModulesStatus = 10,        ///< Get modules status
#endif    
};
