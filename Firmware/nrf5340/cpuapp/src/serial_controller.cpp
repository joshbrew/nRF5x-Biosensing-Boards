#include "serial_controller.hpp"

#include <algorithm>
#include <string.h>

#include <sys/crc.h>

#include <stdio.h>
#include <device.h>
#include <drivers/uart.h>
#include <zephyr.h>
#include <sys/ring_buffer.h>

#include <usb/usb_device.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(usb_serial, LOG_LEVEL_INF);

namespace
{
    constexpr static size_t metadataSize = 5;
    constexpr static size_t headerSize = 4;
    constexpr static size_t smallPacketSize = 2;
    constexpr static size_t minFullPacketSize = 6;

    constexpr static size_t packetHeader0 = 0;
    constexpr static size_t packetHeader1 = 1;
    constexpr static size_t packetHeaderMessageId = 2;
    constexpr static size_t packetHeaderMessageLength = 3;

    constexpr static uint8_t headerValue = 0xF0;
    constexpr static uint8_t headerNegValue = 0xB0;
    constexpr static uint8_t headerAckValue = 0xA0;

    constexpr static uint8_t crcPolynom = 0x31;
    constexpr static uint8_t crcInitilalValue = 0xFF;

    /**
     * @brief Command execution step
     */
    enum class CommandState
    {
        SendCommand,  ///< Sendind command via uart
        WaitResponse, ///< Getting response
        Done,         ///< Command is completed
    };

    constexpr static int serialPortStackSize = 512;
    constexpr static int serialPortTaskPriority = 1;

    K_THREAD_STACK_DEFINE(my_stack_area, serialPortStackSize); ///< Serial controller working task stack
    k_thread worker;                                           ///< worker thread

    std::atomic<TransferStatus> serialStatus; ///< Status of last operation
}

/**
 * @brief Construct Serial Controller.
 * @note Only one instance could be constructed
 */
SerialController::SerialController()
{
    LOG_DBG("Serial Controller Constructor!");
    k_sem_init(&rxSem, 0, 1);
    k_sem_init(&txSem, 0, 1);

    serialStatus.store(TransferStatus::Ok, std::memory_order_relaxed);
    serial_is_initialized_.store(false, std::memory_order_relaxed);
}

/**
 * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
 * could not be done in constructor
 */
void SerialController::Initialize()
{
    
    uint32_t baudrate, dtr = 0U;
	int ret;

    k_msgq_init(&messageQueue, static_cast<char *>(static_cast<void *>(buffer)), sizeof(SerialTransfer *), MaxEntryCount);

    // Start working thread
    k_thread_create(&worker, my_stack_area, K_THREAD_STACK_SIZEOF(my_stack_area),
                    &SerialController::WorkingThread, this, nullptr, nullptr, serialPortTaskPriority, 0, K_NO_WAIT);

	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev)) {
		LOG_ERR("CDC ACM device not ready");
		return;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
}

/**
 * @brief Queues transfer via serial port
 * 
 * @param task queued task
 * @return true if task was successfully added to queue, false owtherwise
 */
bool SerialController:: QueueTransfer(SerialTransfer *task)
{
    task->completed.store(false, std::memory_order_release);

    int status = k_msgq_put(&messageQueue, &task, K_NO_WAIT);

    return status == 0;
}

/**
 * @brief Get status of last operation
 * 
 * @return Status, 0 for no errors
 */
uint8_t SerialController::GetStatus()
{
    TransferStatus status = serialStatus.load(std::memory_order_relaxed);

    return static_cast<uint8_t>(status);
}

/**
 * @brief Thread used to manage serial controller task working queue and controls high level transfers.
 * 
 * @param data pointer to current instance of Serial controller
 */
void SerialController::WorkingThread(void *data, void *, void *)
{
    uint32_t baudrate, dtr = 0U;
	int ret;

    SerialController *self = static_cast<SerialController *>(data);

    //uart_callback_set(self->uartDevice, &SerialController::SerialPortCallback, self);
    //uart_irq_callback_set(self->dev, &SerialController::interrupt_handler);
    uart_irq_callback_user_data_set(self->dev, &SerialController::interrupt_handler, self);

	/* Enable rx interrupts */
	//uart_irq_rx_enable(self->dev);

    SerialTransfer *currentTask;

	LOG_DBG("Wait for DTR");

	while (true) {
		uart_line_ctrl_get(self->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(500));
		}
	}

	LOG_DBG("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(self->dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(self->dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(self->dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate detected: %d", baudrate);
	}
    self->serial_is_initialized_.store(true, std::memory_order_relaxed);    

    for (;;)
    {
        k_msgq_get(&self->messageQueue, &currentTask, K_FOREVER);

        TransferStatus status = TransferStatus::Ok;

        CommandState commandState = CommandState::SendCommand;

        k_sem_reset(&self->rxSem);
        k_sem_reset(&self->txSem);
        //LOG_DBG("SerialController::WorkingThread");
        self->SendPacket(*currentTask->request);

        serialStatus.store(status, std::memory_order_relaxed);

        currentTask->completed.store(true, std::memory_order_release);

        if (currentTask->callback.handler)
        {
            k_work_submit(&currentTask->callback);
        }
    }
}

/**
 * @brief Prepare message buffer and send it via UART
 * 
 * @param packet serial packed to send
 * @return TransferStatus transfer result
 */
TransferStatus SerialController::SendPacket(const SerialPacket &packet)
{
    auto packetSize = SerializeCommand(packet);

    return SendInternal(packetSize);
}

/**
 * @brief Send Neg message.
 * 
 * @return TransferStatus transfer status
 */
TransferStatus SerialController::SendNeg()
{
    sendBuffer[packetHeader0] = headerNegValue;
    sendBuffer[packetHeader1] = headerNegValue;

    return SendInternal(smallPacketSize);
}

/**
 * @brief Sends prepared send buffer.
 * 
 * @param packetSize number of bytes to send
 * 
 * @return TransferStatus transfer status. Return TransferStatus::Timeout if timeout event occurs.
 */
TransferStatus SerialController::SendInternal(size_t packetSize)
{
    uint8_t dummy;
    //while (uart_poll_in(uartDevice, &dummy) == 0); // clear rx queue

    uart_irq_tx_enable(dev);

    uart_fifo_fill(dev, sendBuffer, packetSize);

    int take_result = k_sem_take(&txSem, transferTimeout);

    if (take_result == -EAGAIN)
    {
        //uart_tx_abort(uartDevice);
        LOG_ERR("ERROR: taking semaphore");
        return TransferStatus::Timeout;
    }

    return TransferStatus::Ok;
}

/**
 * @brief Handles receive from connected UART
 * 
 * @param packet reference where received data should be stored
 * @return TransferStatus Transfer status.
 */
TransferStatus SerialController::Receive(SerialPacket &packet)
{
    int take_result = k_sem_take(&rxSem, transferTimeout);

    if (take_result == -EAGAIN)
    {
        uart_rx_disable(uartDevice);
        return TransferStatus::Timeout;
    }

    TransferStatus result = ParseBuffer(packet);

    return result;
}

/**
 * @brief Serialize packet into internal send buffer
 * 
 * @param packet packet to serialize
 * @return number of bytes in serial packet.
 */
size_t SerialController::SerializeCommand(const SerialPacket &packet)
{
    sendBuffer[packetHeader0] = headerValue;
    sendBuffer[packetHeader1] = headerValue;
    sendBuffer[packetHeaderMessageId] = packet.messageId;
    sendBuffer[packetHeaderMessageLength] = packet.length;
    memcpy(sendBuffer + headerSize, packet.dataPtr, packet.length);

    uint8_t crc = crc8(sendBuffer, packet.length + headerSize, crcPolynom, crcInitilalValue, false);

    sendBuffer[headerSize + packet.length] = crc;
    return packet.length + metadataSize;
}

/**
 * @brief Parse receive buffer and if receive buffer contains valid data writes them into output packet
 * 
 * @param packet reference to Serial port packet where response should be written.
 * @return TransferStatus transfer status
 */
TransferStatus SerialController::ParseBuffer(SerialPacket &packet)
{
    if (recvLength < smallPacketSize)
    {
        return TransferStatus::Error;
    }

    if (recvBuffer[packetHeader0] == headerAckValue && recvBuffer[packetHeader1] == headerAckValue)
    {
        packet.length = 0;
        return TransferStatus::Ack;
    }

    if (recvBuffer[packetHeader0] == headerNegValue && recvBuffer[packetHeader1] == headerNegValue)
    {
        return TransferStatus::Neg;
    }

    if (recvBuffer[packetHeader0] == headerValue && recvBuffer[packetHeader1] == headerValue)
    {
        if (recvLength < minFullPacketSize)
        {
            return TransferStatus::Neg;
        }

        uint8_t messageId = recvBuffer[packetHeaderMessageId];
        size_t messageLength = recvBuffer[packetHeaderMessageLength];

        if ((messageLength + metadataSize) != recvLength)
        {
            return TransferStatus::Error;
        }

        uint8_t crc = crc8(recvBuffer, messageLength + headerSize, crcPolynom, crcInitilalValue, false);

        if (crc != recvBuffer[messageLength + headerSize])
        {
            return TransferStatus::Error;
        }

        if (packet.dataPtr)
        {
            packet.length = std::min(static_cast<uint8_t>(messageLength), packet.length);
            memcpy(packet.dataPtr, recvBuffer + headerSize, packet.length);
        }

        packet.messageId = messageId;

        return TransferStatus::Ok;
    }

    return TransferStatus::Error;
}

/**
 * @brief Serial port event callback. called every time, state of the uart port. Used to track buffers 
 * and enables data receive after command is sent via serial port.
 * 
 * @param dev  serial port device 
 * @param evt  serial port event
 * @param data pointer to instance of current instance of the serial port controller.
 */
void SerialController::SerialPortCallback(const device *dev, uart_event *evt, void *data)
{
    SerialController *self = static_cast<SerialController *>(data);

    switch (evt->type)
    {
    case UART_TX_DONE:
        self->recvLength = 0;
        uart_rx_enable(self->uartDevice, self->recvBuffer, sizeof(self->recvBuffer), 1);
        k_sem_give(&self->txSem);
        break;

    case UART_RX_RDY:
        uart_rx_disable(dev);
        self->recvLength += evt->data.rx.len;
        break;

    case UART_RX_BUF_RELEASED:
        k_sem_give(&self->rxSem);
        break;

    default:
        break;
    }
}

void SerialController::interrupt_handler(const struct device *dev, void *user_data)
{
	//ARG_UNUSED(user_data);
    SerialController *self = static_cast<SerialController *>(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {        
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;
            //LOG_DBG("Ready to send back!");
            uart_irq_tx_disable(dev);
          
		}

        if (uart_irq_tx_complete(dev)) {
            //LOG_DBG("TX Complete!");
            k_sem_give(&self->txSem);
            //uart_irq_tx_disable(dev);
        }

	}
}

bool SerialController::IsInitialized(){
    bool status;
    status = serial_is_initialized_.load(std::memory_order_relaxed);
    return status;
}