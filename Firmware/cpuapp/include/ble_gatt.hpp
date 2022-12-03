#pragma once

#include <functional>

#include <bluetooth/gatt.h>
#include <sys/atomic.h>

namespace Bluetooth::Gatt
{

    /**
     * @brief State of the ADS131M08 Notifications.
     */
    extern atomic_t ads131m08NotificationsEnable;

    /**
     * @brief State of the ADS131M08_1 Notifications.
     */
    extern atomic_t ads131m08_1_NotificationsEnable;

    /**
     * @brief State of the MAX30102 Notifications.
     */
    extern atomic_t max30102NotificationsEnable;

    /**
     * @brief State of the MPU6050 Notifications.
     */
    extern atomic_t mpu6050NotificationsEnable;

    /**
     * @brief State of the MPU6050 Notifications.
     */
    extern atomic_t bme280NotificationsEnable;

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
    constexpr static int CharacteristicMax30102Data = 7;

    /**
     * @brief Index of the Gatt MPU6050 Data characteristic in service characteristic table
     */
    constexpr static int CharacteristicMpu6050Data = 10;

    /**
     * @brief Index of the Gatt ADS131M08 Data characteristic in service characteristic table
     */
    constexpr static int CharacteristicAds131_1_Data = 13;

    /**
     * @brief Index of the Gatt ADS131M08 Data characteristic in service characteristic table
     */
    constexpr static int CharacteristicBme280Data = 16;

    /**
     * @brief Callback called when Bluetooth is initialized. Starts BLE server
     * 
     * @param err bluetooth initialization error code
     */
    void OnBluetoothStarted(int err);

}
