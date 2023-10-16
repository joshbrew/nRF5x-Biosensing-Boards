#include "usb_comm_handler.hpp"

#include <zephyr.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(UsbCommHandler, LOG_LEVEL_INF);

namespace
{
    /**
     * @brief define memory heap for uart transport.
     */

    K_HEAP_DEFINE(heapBuffers, 4096);

}

/**
 * @brief Construct a new Uart Commands Transport object.
 * 
 * @param serial Serial port controller.
 */
UsbCommHandler::UsbCommHandler(SerialController &serial)
    : serial(serial)
{
    // Prepare uart transfer buffers
    for (size_t i = 0; i < maxCommands; ++i)
    {
        transfersBuffer[i].request = &sendPackets[i];
        transfersBuffer[i].response = &recvPackets[i];

        Release(&transfersBuffer[i]);
    }
}

/**
 * @brief Initialization function. Used to perform actual initialization. Because of software stack initialization
 * could not be done in constructor
 */
void UsbCommHandler::Initialize()
{
    LOG_DBG("UsbCommHandler Initialization...");
}

/**
 * @brief Callback called by serial controller when command is completed. Releases acuired command resources.
 * @warning Callback is executed in system working thread callback
 * 
 * @param work worker
 */
void UsbCommHandler::CommandCompletedCallback(k_work *work)
{
    SerialTransfer *task = CONTAINER_OF(work, SerialTransfer, callback);
    UsbCommHandler *self = static_cast<UsbCommHandler *>(task->context);
    //LOG_INF("CommandCompletedCallback!");

    k_heap_free(&heapBuffers, task->request->dataPtr);
    k_heap_free(&heapBuffers, task->response->dataPtr);

    self->Release(task);
}

void UsbCommHandler::SendAccelSamples(const uint8_t *buffer, size_t length){
    
    if(serial.IsInitialized()){
        LOG_INF(".");
        SerialTransfer *transfer = CreateTransferFrom(SensorId::Mpu6050, buffer, length, 0);

        QueueTransfer(transfer);   
    } 
}

void UsbCommHandler::SendMagnetometerSamples(const uint8_t *buffer, size_t length){
    
    if(serial.IsInitialized()){
        LOG_INF("=");
        SerialTransfer *transfer = CreateTransferFrom(SensorId::Qmc5883l, buffer, length, 0);

        QueueTransfer(transfer);   
    } 
}

void UsbCommHandler::SendMax30102Samples(const uint8_t *buffer, size_t length){
    
    if(serial.IsInitialized()){
        LOG_INF("*");
        SerialTransfer *transfer = CreateTransferFrom(SensorId::Max30102, buffer, length, 0);

        QueueTransfer(transfer);   
    } 
}

void UsbCommHandler::SendBme280Samples(const uint8_t *buffer, size_t length){
    
    if(serial.IsInitialized()){
        LOG_INF("/");
        
        SerialTransfer *transfer = CreateTransferFrom(SensorId::Bme280, buffer, length, 0);

        QueueTransfer(transfer);   
    } 
}

void UsbCommHandler::SendAds131m08Samples(const uint8_t *buffer, size_t length, uint8_t sensor_id){
        
    SensorId ads131m08_id;
    
    if(serial.IsInitialized()){    
        LOG_INF("-");
        if(sensor_id == 0){
            ads131m08_id = SensorId::Ads131m08_0;
        } else {
            ads131m08_id = SensorId::Ads131m08_1;
        }          
        SerialTransfer *transfer = CreateTransferFrom(ads131m08_id, buffer, length, 0);
        
        QueueTransfer(transfer);    
    }
}

/**
 * @brief Queues transfer to UART, and if autoReleaseOnError is set and any error occurs releases transfer and its
 *        allocated buffers
 * 
 * @param transfer            tranfer to send
 * @param autoReleaseOnError  set to true if transfer should be autoreleased on error
 * @return true               if message was sent successfully, false otherwise
 */
bool UsbCommHandler::QueueTransfer(SerialTransfer *transfer, bool autoReleaseOnError)
{
    bool result = serial.QueueTransfer(transfer);

    if (!result && autoReleaseOnError)
    {
        k_heap_free(&heapBuffers, transfer->request->dataPtr);
        k_heap_free(&heapBuffers, transfer->response->dataPtr);

        Release(transfer);
    }
    return result;
}

/**
 * @brief Creates and initializes Serial Transfer object used to send command to stm8
 * 
 * @param messageId         message id
 * @param req               request buffer 
 * @param reqLen            request buffer length. Set this field to 0 if request contains no data
 * @param respMaxLen        maximum reponse buffer length. Set this field to 0 if no responce from stm8 is expected
 * @return SerialTransfer*  constructed transfer, or nullptr if transfer creation was unsuccessful 
 */
SerialTransfer *UsbCommHandler::CreateTransferFrom(SensorId messageId, const uint8_t *req, size_t reqLen, size_t respMaxLen)
{
    void *request = nullptr;
    uint8_t requestLen = 0;
    void *response = nullptr;
    uint8_t responseLen = 0;

    if (reqLen != 0)
    {
        request = k_heap_alloc(&heapBuffers, reqLen, K_NO_WAIT);
        if (request == nullptr)
        {
            LOG_ERR("Cannot allocate transfer buffer. Possible buffer leak");
            return nullptr;
        }

        memcpy(request, req, reqLen);
        requestLen = reqLen;
    }

    if (respMaxLen != 0)
    {
        response = k_heap_alloc(&heapBuffers, respMaxLen, K_NO_WAIT);
        if (response == nullptr)
        {
            LOG_ERR("Cannot allocate transfer buffer. Possible buffer leak");

            k_heap_free(&heapBuffers, request);

            return nullptr;
        }
        responseLen = respMaxLen;
    }

    SerialTransfer *transfer = Allocate();
    if (transfer == nullptr)
    {
        k_heap_free(&heapBuffers, request);
        k_heap_free(&heapBuffers, response);

        return nullptr;
    }

    transfer->request->dataPtr = static_cast<uint8_t *>(request);
    transfer->request->length = requestLen;
    transfer->request->messageId = static_cast<uint8_t>(messageId);

    transfer->response->dataPtr = static_cast<uint8_t *>(response);
    transfer->response->length = responseLen;
    transfer->context = this;
    transfer->callback = Z_WORK_INITIALIZER(&UsbCommHandler::CommandCompletedCallback);

    return transfer;
}

/**
 * @brief Allocates serial transfer to execute command
 * 
 * @return SerialTransfer* serial transfers buffer.
 */
SerialTransfer *UsbCommHandler::Allocate()
{
    int index = tailIndex.load(std::memory_order_acquire);
    int nextTail = (index + 1) % capacity;

    if (tailIndex == headIndex.load(std::memory_order_consume))
    {
        return nullptr;
    }

    SerialTransfer *retval = transfers[index];

    tailIndex.store(nextTail, std::memory_order_release);
    return retval;
}

/**
 * @brief Release serial transfer after command execution is complete
 * 
 * @param transfer transfer to release
 */
void UsbCommHandler::Release(SerialTransfer *buffer)
{
    int currentHead = headIndex.load(std::memory_order_acquire);
    int nextHead = (currentHead + 1) % capacity;
    if (nextHead == tailIndex.load(std::memory_order_consume))
    {
        LOG_ERR("Unreachable");
    }

    transfers[currentHead] = buffer;
    headIndex.store(nextHead, std::memory_order_release);
}
