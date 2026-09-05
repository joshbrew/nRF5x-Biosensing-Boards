#pragma once

#include <functional>

namespace Bluetooth
{
    /**
     * @brief Function used to setup BLE Service
     * 
     * @return BLE error code 
     */
    int SetupBLE();

    /**
     * @brief Function used to notify connected clients when new Sensor (ADS131M08) data are available
     * 
     * @param data Accelerometer measurement data
     */
    void SensorNotify(const uint8_t* data, const uint8_t len);

}
