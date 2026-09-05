#pragma once

#include <functional>

#include <bluetooth/gatt.h>
#include <sys/atomic.h>

namespace Bluetooth::Gatt
{

    /**
     * @brief State of the accelerometerNotificatios.
     */
    extern atomic_t ads131m08NotificationsEnable;

    /**
     * @brief GATT service
     */
    extern const bt_gatt_service_static bt832a_svc;

    /**
     * @brief Index of the Gatt SensorData characteristic in service characteristic table
     */
    constexpr static int CharacteristicSensorData = 4;

    /**
     * @brief Callback called when Bluetooth is initialized. Starts BLE server
     * 
     * @param err bluetooth initialization error code
     */
    void OnBluetoothStarted(int err);

}
