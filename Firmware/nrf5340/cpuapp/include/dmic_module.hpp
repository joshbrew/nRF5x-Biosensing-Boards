#pragma once

#include <string.h>
#include <nrf.h>
#include <kernel.h>

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>

#include <sys/util.h>
#include <atomic>
#include "ble_types.hpp"
#include "ble_commands.hpp"
#include "device_string.hpp"

/*************************/
#include <zephyr.h>
#include <device.h>
#include <audio/dmic.h>
#include "arm_math.h"
#include "dsp/transform_functions.h"

#include <logging/log.h>
#include <sys/printk.h>

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

#define FFT_LENGTH 32

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      8

namespace
{
    constexpr static int stackSizeDmic = 2048;           ///< Worker thread size
    constexpr static int taskPriorityDmic = 1;           ///< Worker thread priority
    K_THREAD_STACK_DEFINE(pollStackAreaDmic, stackSizeDmic); ///< Worker thread stack
} // namespace

/**
 * @brief DmicModule driver
 */
class DmicModule {
    
public:
    /**
     * @brief Construct a new DmicModule object
     */
    DmicModule();

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     * @return Status, 0 for no errors
     */
    int Initialize();

    /**
     * @brief Start taking Audio samples
     */
    int StartSampling();

    /**
     * @brief Stop taking Audio samples
     */
    int StopSampling();

private:
    /**
     * @brief Called when trigger mode is received via BLE
     * 
     * @param buffer receviced buffer
     * @param length buffer length
     * @param offset data offset
     * 
     * @return true if command was processed succesfully
     */
    bool OnBleCommand(const uint8_t* buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset);
    int do_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg, size_t block_count);

    static void WorkingThreadDmic(void *data, void *, void *);
    k_thread worker;     ///< Worker thread
    void *mem_blocks;
    const struct device* dmic_dev; ///< Logical device
};
