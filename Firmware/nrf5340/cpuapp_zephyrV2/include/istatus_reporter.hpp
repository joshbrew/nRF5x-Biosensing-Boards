#pragma once

#include <stdint.h>

/**
 * @brief Status reporter interface
 */
struct IStatusReporter
{
    /**
     * @brief Get current status
     * 
     * @return Status, 0 for no errors
     */
    virtual uint8_t GetStatus() = 0;
};