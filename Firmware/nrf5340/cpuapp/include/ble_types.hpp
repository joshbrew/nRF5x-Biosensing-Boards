#pragma once

#include <functional>

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

    static_assert(sizeof(CommandKey) == sizeof(uint16_t));


    /**
     * @brief Ble action type
     */
    using BleControlAction = std::function<bool(const uint8_t*, CommandKey, BleLength, BleOffset)>;

}
