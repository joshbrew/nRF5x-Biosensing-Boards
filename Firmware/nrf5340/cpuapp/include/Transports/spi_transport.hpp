#if 0
#pragma once

#include <atomic>

#include <zephyr/drivers/spi.h>

#define ADS131_DRV DT_NODELABEL(ads131drv) ///< ADS131M08 device

/**
 * @brief SPI transport for BMI088 accelerometer
 */
class SPIAccelTransport
{
    constexpr static uint8_t readWriteBit = 0x80; ///< Read/write bit for first byte of accelerometer 16-bit protocol

public:
    /**
     * @brief Initialization function used to intialize when underlayer infrastructure is ready
     */
    void Initialize()
    {
//#if DT_NODE_HAS_STATUS(ADS131_DRV, okay) && DT_SPI_DEV_HAS_CS_GPIOS(ADS131_DRV)
        // Try to bind chip select device
        csConfig.gpio_dev = device_get_binding(DT_SPI_DEV_CS_GPIOS_LABEL(ADS131_DRV));
        csConfig.delay = 0;
        csConfig.gpio_pin = DT_SPI_DEV_CS_GPIOS_PIN(ADS131_DRV);
        csConfig.gpio_dt_flags = DT_SPI_DEV_CS_GPIOS_FLAGS(ADS131_DRV);

        // Bind SPI device only if chip select device successfully binded
        if (csConfig.gpio_dev)
        {
            spiConfig.frequency = DT_PROP(ADS131_DRV, spi_max_frequency);
            spiConfig.operation = SPI_WORD_SET(8);
            spiConfig.slave = DT_REG_ADDR(ADS131_DRV);
            spiConfig.cs = &csConfig;

            spiDevice = device_get_binding(DT_LABEL(DT_BUS(ADS131_DRV)));
        }
//#endif //DT_NODE_HAS_STATUS(ADS131_DRV, okay) && DT_SPI_DEV_HAS_CS_GPIOS(ADS131_DRV)

        if (spiDevice == nullptr)
        {
            deviceStatus.store(-1, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Write device Register
     * 
     * @param registerId Id of the register to write
     * @param value value to write
     */
    void WriteRegister(uint8_t registerId, uint8_t value)
    {
        if (spiDevice != nullptr)
        {
            int status;
            registerId &= ~readWriteBit;
            const struct spi_buf txBuffers[] = {
                {
                    .buf = &registerId,
                    .len = 1,
                },
                {
                    .buf = &value,
                    .len = 1,
                },
            };
            const struct spi_buf_set txSet = {
                .buffers = txBuffers,
                .count = sizeof(txBuffers) / sizeof(spi_buf),
            };

            status = spi_write(spiDevice, &spiConfig, &txSet);

            deviceStatus.store(status, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Read device register
     * 
     * @param registerId Id of the register to read
     * @return uint8_t hardware register id
     */
    uint8_t ReadRegister(uint8_t registerId)
    {
        uint8_t value = 0;

        if (spiDevice != nullptr)
        {
            int status;
            registerId |= readWriteBit;
            const struct spi_buf txBuffers[] = {
                {
                    .buf = &registerId,
                    .len = 1,
                },
                {
                    .buf = NULL,
                    .len = 2,
                },
            };
            const struct spi_buf rxBuffers[] = {
                {
                    .buf = NULL,
                    .len = 2,
                },
                {
                    .buf = &value,
                    .len = 1,
                },
            };
            const struct spi_buf_set txSet = {
                .buffers = txBuffers,
                .count = sizeof(txBuffers) / sizeof(spi_buf),
            };
            const struct spi_buf_set rxSet = {
                .buffers = rxBuffers,
                .count = sizeof(rxBuffers) / sizeof(spi_buf),
            };

            status = spi_transceive(spiDevice, &spiConfig, &txSet, &rxSet);

            deviceStatus.store(status, std::memory_order_relaxed);
        }

        return value;
    }

    /**
     * @brief Update hardware register. Reads hardware register updated register bits selected with mask
     *        and writes updated register value back
     * 
     * @param registerId Register ID to update
     * @param mask Bitmask of the register to update
     * @param value value for register
     */
    void UpdateRegister(uint8_t registerId, uint8_t mask, uint8_t value)
    {
        uint8_t newValue;
        uint8_t oldValue = ReadRegister(registerId);

        newValue = (oldValue & ~mask) | (value & mask);

        if (newValue != oldValue)
        {
            WriteRegister(registerId, newValue);
        }
    }

    /**
     * @brief Read hardware register batch from device
     * 
     * @param registerId Start register ID 
     * @param data pointer to data object, where register batch should be stored
     * @param size number of registers to read
     */
    void ReadRegisters(uint8_t registerId, uint8_t *data, int size)
    {
        if (spiDevice != nullptr)
        {
            int status;
            registerId |= readWriteBit;
            const struct spi_buf txBuffers[] = {
                {
                    .buf = &registerId,
                    .len = 1,
                },
                {
                    .buf = NULL,
                    .len = static_cast<size_t>(size + 1),
                },
            };
            const struct spi_buf rxBuffers[] = {
                {
                    .buf = NULL,
                    .len = 2,
                },
                {
                    .buf = data,
                    .len = static_cast<size_t>(size),
                },
            };
            const struct spi_buf_set txSet = {
                .buffers = txBuffers,
                .count = sizeof(txBuffers) / sizeof(spi_buf),
            };
            const struct spi_buf_set rxSet = {
                .buffers = rxBuffers,
                .count = sizeof(rxBuffers) / sizeof(spi_buf),
            };

            status = spi_transceive(spiDevice, &spiConfig, &txSet, &rxSet);

            deviceStatus.store(status, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Get device status
     * 
     * @return Current status, 0 for no errors
     */
    uint8_t GetStatus()
    {
        int status = deviceStatus.load(std::memory_order_relaxed);

        return static_cast<uint8_t>(-status);
    }

private:
    std::atomic<int> deviceStatus = 0; ///< SPI Device status
    const device *spiDevice = nullptr; ///< Logical SPI device

    struct spi_cs_control csConfig; ///< Chip select config
    struct spi_config spiConfig;    ///< SPI transport config
};


#endif