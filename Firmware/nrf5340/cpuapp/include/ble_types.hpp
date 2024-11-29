#pragma once

#include <functional>
#include <bluetooth/bluetooth.h>

namespace Bluetooth
{
    /**
     * @brief Structure to contain Ble Length. Acts as strong typedef
     */
    struct BleLength
    {
        uint16_t value;
    };

    static_assert(sizeof(BleLength) == sizeof(uint16_t));

    /**
     * @brief Structure to contain Ble Offset. Acts as strong typedef
     */
    struct BleOffset
    {
        uint16_t value;
    };

    static_assert(sizeof(BleOffset) == sizeof(uint16_t));

    /**
     * @brief Structure to hold Command Key field. Acts as a strong typedef
     */
    struct CommandKey
    {
        uint8_t key[2];    
    };

    /**
     * @brief Structure to hold the iBeacon details
     */
    struct iBeacon
    {
        uint8_t  addr[BT_ADDR_SIZE];
        int8_t rssi;
        uint8_t uuid[16];
        uint8_t major[2];
        uint8_t minor[2];
        int8_t tx_pwr;
    };

    static_assert(sizeof(CommandKey) == sizeof(uint16_t));


    /**
     * @brief Ble action type
     */
    using BleControlAction = std::function<bool(const uint8_t*, CommandKey, BleLength, BleOffset)>;

}
