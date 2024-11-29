#pragma once

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <atomic>
#include <stdlib.h>
#include <logging/log.h>

#include "ble_service.hpp"
#include "usb_comm_handler.hpp"
#include "ble_types.hpp"
#include "ble_commands.hpp"

class UsbCommHandler;

namespace
{
    constexpr static int stackSize = 1024;           ///< Worker thread size
    constexpr static int taskPriority = 7;           ///< Worker thread priority
    K_THREAD_STACK_DEFINE(pollStackArea, stackSize); ///< Worker thread stack

struct bme280_data {
	/* Compensation parameters. */
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;

	/* Compensated values. */
	int32_t comp_temp;
	uint32_t comp_press;
	uint32_t comp_humidity;

	/* Carryover between temperature and pressure/humidity compensation. */
	int32_t t_fine;

	uint8_t chip_id;
};    
}

/**
 * @brief Bme280 driver
 */
class Bme280 {   
  
    constexpr static auto pollPeriod = 300;  ///< BME280 polling period in milliseconds
    constexpr static uint8_t bme280_id = 0x60;
    constexpr static uint8_t bmp280_id_sample_1 = 0x56;
    constexpr static uint8_t bmp280_id_sample_2 = 0x57;
    constexpr static uint8_t bmp280_id_mp = 0x58;

public:
    /**
     * @brief Construct a new Bme280 object
     */
    Bme280(UsbCommHandler &controller);

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     * @return Status, 0 for no errors
     */
    int Initialize();    

    /**
     * @brief Start taking BME280 samples (ambient tempeature, pressure, himidity)
     */
    void StartSampling();    

    /**
     * @brief Stop taking BME280 samples (ambient tempeature, pressure, himidity)
     */
    void StopSampling();

    /**
     * @brief Check if either BME280 or BMP280 device is connected to I2C bus
     * @return TRUE if either of BME280 or BMP280 is on the bus, FALSE otherwise
     */
    bool BmX280IsOnI2cBus(); 

    /**
     * @brief Check if BME280 device is connected to I2C bus
     * @return TRUE if BME280 is on the bus, FALSE otherwise
     */
    bool Bme280IsOnI2cBus(); 

private:
    /**
     * @brief Main working thread. Used to perform accelerometer polling.
     * Due possible blocking during accelerometer measurments reading
     * it required a separate stack to not break main BLE stack state machine
     * 
     * @param data pointer to this
     */
    static void WorkingThread(void *data, void *, void *){
        Bme280 *self = static_cast<Bme280 *>(data);
        int ret = 0;        
        self->tx_buf[0] = self->bmx_id;

        for (;;)
        {
            k_sem_take(&self->pollSemaphore, K_FOREVER);

            ret = sensor_sample_fetch(self->bme280_dev);
            ret += sensor_channel_get(self->bme280_dev, SENSOR_CHAN_AMBIENT_TEMP, &self->temperature_channel);
            ret += sensor_channel_get(self->bme280_dev, SENSOR_CHAN_PRESS, &self->pressure_channel);
		    
            if(self->Bme280IsOnI2cBus()){ // If BME280 is on I2C Buss
               ret += sensor_channel_get(self->bme280_dev, SENSOR_CHAN_HUMIDITY, &self->humidity_channel);

                if (ret == 0){
                    memcpy((self->tx_buf + 24*self->sample_cnt + 2), &self->temperature_channel.val1, sizeof(self->temperature_channel));
                    memcpy((self->tx_buf + 24*self->sample_cnt + 10), &self->pressure_channel.val1, sizeof(self->pressure_channel));
                    memcpy((self->tx_buf + 24*self->sample_cnt + 18), &self->humidity_channel.val1, sizeof(self->humidity_channel));

                    self->sample_cnt++;

                    if(self->sample_cnt == 3){
                        self->sample_cnt = 0;
                        self->tx_buf[1] = self->packet_cnt;
                        self->packet_cnt++;
                        self->serialHandler.SendBme280Samples(self->tx_buf, 74);
                        Bluetooth::Bme280Notify(self->tx_buf, 74); 
                    }
                } else {

                }           
            } else {  // BMP280 is on the bus. Don't fetch Humidity data.
                if (ret == 0){
                    memcpy((self->tx_buf + 16*self->sample_cnt + 2), &self->temperature_channel.val1, sizeof(self->temperature_channel));
                    memcpy((self->tx_buf + 16*self->sample_cnt + 10), &self->pressure_channel.val1, sizeof(self->pressure_channel));

                    self->sample_cnt++;

                    if(self->sample_cnt == 3){
                        self->sample_cnt = 0;
                        self->tx_buf[1] = self->packet_cnt;
                        self->packet_cnt++;
                        self->serialHandler.SendBme280Samples(self->tx_buf, 50);
                        Bluetooth::Bme280Notify(self->tx_buf, 50); 
                    }
                } else {

                }           
            }

        }
    }

    /**
     * @brief Reset Bme280 device
     */
    void Reset();

    /**
     * @brief Get a device structure from a devicetree node with compatible
     * "bosch,bme280". (If there are multiple, just pick one.)
     */
    const struct device * get_bme280_device(void);

    /**
     * @brief Shutdown Bme280 device
     */
    void Shutdown();

    /**
     * @brief Wakeup Bme280 device
     */
    void Wakeup();

    /**
     * @brief Read ID register and figure out whether we have BMP280 or BME280 on I2C bus
     */
    void GetChipId(const struct device *dev);

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

    /**
     * @brief Timer handler. Used to queue Reading of the BME280 data.
     * 
     * @param tmr timer object
     * @warning Called at ISR Level, no actual workload should be implemented here
     */
    static void TimerHandler(k_timer *tmr){
        Bme280 *self = CONTAINER_OF(tmr, Bme280, timer);
        k_sem_give(&self->pollSemaphore);
    }

    struct bme280_data * ToData(const struct device *dev);

    const struct device * bme280_dev;
    uint8_t bmx_id; 
    uint8_t sample_cnt;
    uint8_t packet_cnt;
    std::atomic<bool> bme280_is_on_i2c_bus_; ///< Device status
    std::atomic<bool> bmp280_is_on_i2c_bus_; ///< Device status
    UsbCommHandler &serialHandler; ///< USB communication controller
    uint8_t tx_buf[73] = {};
    k_timer timer;       ///< Timer object
    k_sem pollSemaphore; ///< Semaphore used to accelerometer polling
    k_thread worker;     ///< Worker thread
    struct sensor_value temperature_channel; ///< Value taken from one of the BME280 ambient temperature channel
    struct sensor_value pressure_channel; ///< Value taken from one of the BME280 pressure channel
    struct sensor_value humidity_channel; ///< Value taken from one of the BME280 humidity channel
};
