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
     * @brief State of the accelerometerNotificatios.
     */
    extern atomic_t max30102NotificationsEnable;

    /**
     * @brief GATT service
     */
    extern const bt_gatt_service_static bt832a_svc;

    /**
     * @brief Index of the Gatt ADS131M08 Data characteristic in service characteristic table
     */
    constexpr static int CharacteristicAds131Data = 4;

    /**
     * @brief Index of the Gatt MAX30102 Data characteristic in service characteristic table
     */
    constexpr static int CharacteristicMax30102Data = 6;

    /**
     * @brief Callback called when Bluetooth is initialized. Starts BLE server
     * 
     * @param err bluetooth initialization error code
     */
    void OnBluetoothStarted(int err);

}
