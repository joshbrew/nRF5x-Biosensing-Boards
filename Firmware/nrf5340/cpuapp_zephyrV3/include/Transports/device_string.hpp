#pragma once

/**
 * @brief Compile time Device String Template
 */
template <char... chars>
struct DeviceString{
    static char const * c_str() {
        static constexpr char string[]={chars...,'\0'};
        return string;
    }
};
