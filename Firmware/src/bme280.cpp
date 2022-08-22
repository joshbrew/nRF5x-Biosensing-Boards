#include "bme280.hpp"

LOG_MODULE_REGISTER(bme280);

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
        bme280_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
        // Start working thread
        k_thread_create(&worker, pollStackArea, K_THREAD_STACK_SIZEOF(pollStackArea),
                        &WorkingThread, this, nullptr, nullptr, taskPriority, 0, K_NO_WAIT);        
    } else {
        bme280_is_on_i2c_bus_.store(false, std::memory_order_relaxed);
        return -1;
    }

    return ret;
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

void Bme280::StartSampling(){
    // Start BME280 polling
    k_timer_start(&timer, K_MSEC(pollPeriod), K_MSEC(pollPeriod));
}   

void Bme280::StopSampling(){
    k_timer_stop(&timer);
}

bool Bme280::IsOnI2cBus(){
    bool status;
    status = bme280_is_on_i2c_bus_.load(std::memory_order_relaxed);
    return status;
}