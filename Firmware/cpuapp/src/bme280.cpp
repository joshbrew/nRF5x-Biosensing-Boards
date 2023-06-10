#include "bme280.hpp"
// For registering callback
#include "ble_service.hpp"

LOG_MODULE_REGISTER(bme280, LOG_LEVEL_INF);

Bme280::Bme280(UsbCommHandler &controller) : serialHandler(controller) {
    LOG_INF("Bme280 Constructor!");
    k_timer_init(&timer, &Bme280::TimerHandler, nullptr);
    k_sem_init(&pollSemaphore, 0, 1);    
}

int Bme280::Initialize() {

    int ret = 0;
    uint8_t part_id;
    
    LOG_INF("Starting Bme280 Initialization..."); 
    
    packet_cnt = 0;
    sample_cnt = 0;
    bme280_dev = get_bme280_device();

    if (bme280_dev != nullptr){
        GetChipId(bme280_dev);
        // Start working thread
        k_thread_create(&worker, pollStackArea, K_THREAD_STACK_SIZEOF(pollStackArea),
                        &WorkingThread, this, nullptr, nullptr, taskPriority, 0, K_NO_WAIT);        
    } else {
        bme280_is_on_i2c_bus_.store(false, std::memory_order_relaxed);
        bmp280_is_on_i2c_bus_.store(false, std::memory_order_relaxed);
        bmx_id = 0x00;
        return -1;        
    }

    Bluetooth::GattRegisterControlCallback(CommandId::Bme280Cmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    return ret;
}

bool Bme280::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("Bme280 BLE Command received");

    Bluetooth::CommandKey bleCommand;
    memcpy(&bleCommand, &key, sizeof(key));
          
    switch(bleCommand.key[0]){
        case static_cast<uint8_t>(BleCommand::StartSampling):
            StartSampling();
            break;
        case static_cast<uint8_t>(BleCommand::StopSampling):
            StopSampling();
            break;
        
        default:
            break;
    }
    
    return true;
}

void Bme280::GetChipId(const struct device *dev){
    struct bme280_data *data = ToData(dev);

    bmx_id = data->chip_id;
    if(data->chip_id == bme280_id){
        LOG_INF("BME280 on I2C bus!");
        bme280_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
    } else if (data->chip_id == bmp280_id_sample_1 || data->chip_id == bmp280_id_mp){
        LOG_INF("BMP280 on I2C bus!");
        bmp280_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
    }
}
const struct device * Bme280::get_bme280_device(void){
	
    const struct device *dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == nullptr) {
		/* No such node, or the node does not have status "okay". */
		LOG_ERR("\nError: no BME280 device found.\n");
		return nullptr;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);        
		return nullptr;
	}

	LOG_INF("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;    
}

struct bme280_data * Bme280::ToData(const struct device *dev){
    return static_cast<bme280_data *>(dev->data);
}

void Bme280::StartSampling(){
    // Start BME280 polling
    k_timer_start(&timer, K_MSEC(pollPeriod), K_MSEC(pollPeriod));
}   

void Bme280::StopSampling(){
    k_timer_stop(&timer);
}

bool Bme280::BmX280IsOnI2cBus(){
    bool status;
    // Return TRUE if either of BMP280 or BME280 is on I2C bus
    status = (bme280_is_on_i2c_bus_.load(std::memory_order_relaxed) || bmp280_is_on_i2c_bus_.load(std::memory_order_relaxed));
    return status;
}

bool Bme280::Bme280IsOnI2cBus(){
    bool status;
    // Return TRUE if BME280 is on I2C bus
    status = bme280_is_on_i2c_bus_.load(std::memory_order_relaxed);
    return status;
}
