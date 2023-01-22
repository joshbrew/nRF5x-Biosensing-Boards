#pragma once

#include <array>
#include <atomic>
#include <functional>

#include "ble_service.hpp"
#include "serial_controller.hpp"

#include "sensor_id.hpp"

/**
 * @brief USB/UART transport. Used to pass sensor readings through USB link
 */
class UsbCommHandler
{
    /**
     * @brief Maximum number of commands/packets could be send to PC simultaneously.
     */
    constexpr static size_t maxCommands = 20;

public:
    /**
     * @brief Construct a new Uart Commands Transport object.
     * 
     * @param serial Serial port controller.
     */
    UsbCommHandler(SerialController &serial);

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     */
    void Initialize();

    /**
     * @brief API function for sending Acceleromter Data over USB. Prepares command that contains samples from the Accelerometer sensor
     * 
     * @param buffer buffer with message data. Data should be copied before use
     * @param length current data chunk length.
     */
    void SendAccelSamples(const uint8_t *buffer, size_t length);

    /**
     * @brief API function for sending MAX30102 Data over USB. Prepares command that contains samples from the MAX30102 sensor
     * 
     * @param buffer buffer with message data. Data should be copied before use
     * @param length current data chunk length.
     */    
    void SendMax30102Samples(const uint8_t *buffer, size_t length);    
    
    /**
     * @brief API function for sending ADS131M08 Data over USB. Prepares command that contains samples from the ADS131M08 sensor
     * 
     * @param buffer buffer with message data. Data should be copied before use
     * @param length current data chunk length.
     * @param sensor_id ID of the ADS131M08 sensor. We currently have two ADS131M08 sensors attached to I2C bus
     */    
    void SendAds131m08Samples(const uint8_t *buffer, size_t length, uint8_t sensor_id);

    /**
     * @brief API function for sending BME280 Data over USB. Prepares command that contains samples from the BME280 sensor
     * 
     * @param buffer buffer with message data. Data should be copied before use
     * @param length current data chunk length.
     */    
    void SendBme280Samples(const uint8_t *buffer, size_t length); 

private:
    /**
     * @brief Callback called by serial controller when command is completed. Releases acuired command resources.
     * @warning Callback is executed in system working thread callback
     * 
     * @param work worker
     */
    static void CommandCompletedCallback(k_work *work);

    /**
     * @brief Creates and initializes Serial Transfer object used to send command to stm8
     * 
     * @param messageId         message id
     * @param req               request buffer 
     * @param reqLen            request buffer length. Set this field to 0 if request contains no data
     * @param respMaxLen        maximum reponse buffer length. Set this field to 0 if no responce from stm8 is expected
     * @return SerialTransfer*  constructed transfer, or nullptr if transfer creation was unsuccessful 
     */
    SerialTransfer *CreateTransferFrom(SensorId messageId, const uint8_t *req, size_t reqLen, size_t respMaxLen);

    /**
     * @brief Queues transfer to UART, and if autoReleaseOnError is set and any error occurs releases transfer and its
     *        allocated buffers
     * 
     * @param transfer            tranfer to send
     * @param autoReleaseOnError  set to true if transfer should be autoreleased on error
     * @return true               if message was sent successfully, false otherwise
     */
    bool QueueTransfer(SerialTransfer *transfer, bool autoReleaseOnError = false);

    /**
     * @brief Allocates serial transfer to execute command
     * 
     * @return SerialTransfer* serial transfers buffer.
     */
    SerialTransfer *Allocate();

    /**
     * @brief Release serial transfer after command execution is complete
     * 
     * @param transfer transfer to release
     */
    void Release(SerialTransfer *transfer);

    SerialController &serial; ///< Serial controller used to send commands to stm8

    SerialPacket sendPackets[maxCommands];       ///< Serial transfer send buffers
    SerialPacket recvPackets[maxCommands];       ///< Serial transfer receive buffers
    SerialTransfer transfersBuffer[maxCommands]; ///< Serial transfer commands buffers

    constexpr static size_t capacity = 32;            ///< SPSC circular buffer size
    std::array<SerialTransfer *, capacity> transfers; ///< SPSC circular buffer
    std::atomic<int> headIndex;                       ///< SPSC head circular buffer index
    std::atomic<int> tailIndex;                       ///< SPSC tail circular buffer index
};
