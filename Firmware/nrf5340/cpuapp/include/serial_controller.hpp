#pragma once

#include <zephyr/kernel.h>

#include <atomic>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include "istatus_reporter.hpp"

/**
 * @brief Transfer Status
 */
enum class TransferStatus
{
    Ok,      ///< Transfer complete
    Ack,     ///< Ack status was received
    Neg,     ///< Neg status was received
    Error,   ///< Transfer error
    Timeout, ///< Serial port timeout
};

/**
 * @brief Wraps serial port request/responce data
 */
struct SerialPacket
{
    uint8_t *dataPtr;  ///< Pointer to data buffer. set to nullptr for response if data from response is not required.
    uint8_t messageId; ///< Message Id
    uint8_t length;    ///< Data length
};

/**
 * @brief Encapsulates serial port transfer
 */
struct SerialTransfer
{
    SerialPacket *request;       ///< Data packet to be send via serial port
    SerialPacket *response;      ///< Data packet where serial port response will be written
    uint32_t key;                ///< Data key. Could be used to stored additional data
    k_work callback;             ///< Data transfer complete callback. set to nullptr if no callback is required
    void *context;               ///< Transfer context.
    std::atomic<bool> completed; ///< Set to true if when command is completed.
};

/**
 * @brief Serial port communication controller. Encapsulates asyncronus USB/UART communication with the PC.
 */
class SerialController : public IStatusReporter
{
    constexpr static size_t BufferSize = 256; ///< Send and Receive buffer size
    constexpr static int MaxEntryCount = 20;  ///< Maximum number of scheduled serial commands.
    k_timeout_t transferTimeout = K_MSEC(50); ///< Transfer timeout

public:
    /**
     * @brief Construct Serial Controller.
     * @note Only one instance could be constructed
     */
    SerialController();

    /**
     * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
     * could not be done in constructor
     */
    void Initialize();

    /**
     * @brief Queues transfer via serial port
     * 
     * @param task queued task
     * @return true if task was successfully added to queue, false owtherwise
     */
    bool QueueTransfer(SerialTransfer *task);

    /**
     * @brief Check if SerialController is initialized
     *      
     * @return true if initialized, false owtherwise
     */
    bool IsInitialized();

    /**
     * @brief Get status of last operation
     * 
     * @return Status, 0 for no errors
     */
    virtual uint8_t GetStatus() override;

private:
    /**
     * @brief Serial port event callback. called every time, state of the uart port. Used to track buffers 
     * and enables data receive after command is sent via serial port.
     * 
     * @param dev  serial port device 
     * @param evt  serial port event
     * @param data pointer to instance of current instance of the serial port controller.
     */
    static void SerialPortCallback(const device *dev, uart_event *evt, void *data);

    static void interrupt_handler(const struct device *dev, void *user_data);

    /**
     * @brief Thread used to manage serial controller task working queue and controls high level transfers.
     * 
     * @param data pointer to current instance of Serial controller
     */
    static void WorkingThread(void *data, void *, void *);

    /**
     * @brief Prepare message buffer and send it via UART
     * 
     * @param packet serial packed to send
     * @return TransferStatus transfer result
     */
    TransferStatus SendPacket(const SerialPacket &packet);

    /**
     * @brief Send Neg message.
     * 
     * @return TransferStatus transfer status
     */
    TransferStatus SendNeg();

    /**
     * @brief Sends prepared send buffer.
     * 
     * @param packetSize number of bytes to send
     * 
     * @return TransferStatus transfer status. Return TransferStatus::Timeout if timeout event occurs.
     */
    TransferStatus SendInternal(size_t packetSize);

    /**
     * @brief Handles receive from connected UART
     * 
     * @param packet reference where received data should be stored
     * @return TransferStatus Transfer status.
     */
    TransferStatus Receive(SerialPacket &packet);

    /**
     * @brief Serialize packet into internal send buffer
     * 
     * @param packet packet to serialize
     * @return number of bytes in serial packet.
     */
    size_t SerializeCommand(const SerialPacket &packet);

    /**
     * @brief Parse receive buffer and if receive buffer contains valid data writes them into output packet
     * 
     * @param packet reference to Serial port packet where response should be written.
     * @return TransferStatus transfer status
     */
    TransferStatus ParseBuffer(SerialPacket &packet);

    const device *uartDevice;       ///< UART device
    const device *dev;       ///< UART devicedev
    uint8_t recvBuffer[BufferSize]; ///< recevice buffer;
    size_t recvLength;              ///< number of bytes was recived via current transfer
    uint8_t sendBuffer[BufferSize]; ///< send buffer;

    k_sem rxSem; ///< Data received semaphore
    k_sem txSem; ///< Data sent semaphore.

    k_msgq messageQueue;                   ///< Serial port task queue
    SerialTransfer *buffer[MaxEntryCount]; ///< Serial port task buffer
    std::atomic<bool> serial_is_initialized_; ///< Device status
};