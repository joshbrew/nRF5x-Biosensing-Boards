#include <zephyr.h>
#include <string.h>
#include <init.h>

#include <sys/printk.h>
#include <stdlib.h>
#include <logging/log.h>

#include "tlc5940.hpp"
#include "ble_service.hpp"

// For registering callback
#include "ble_service.hpp"

LOG_MODULE_REGISTER(tlc5940, LOG_LEVEL_INF);

Tlc5940::Tlc5940() {
    LOG_INF("Tlc5940 Constructor!");
}

int Tlc5940::Initialize() {

    int ret = 0;
    uint8_t part_id;
    
    LOG_INF("Starting Tlc5940 Initialization..."); 
    
    InitializeGpios();
    SetPin(GSCLK_PIN, 0);
    SetPin(XLAT_PIN, 0);
    SetPin(BLANK_PIN, 1);
    SetPin(SIN_PIN, 0);
    SetPin(SCLK_PIN, 0);

    Bluetooth::GattRegisterControlCallback(CommandId::Tlc5940Cmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    return ret;
}

void Tlc5940::Set(uint8_t channel, uint16_t value){
    
}

void Tlc5940::SetAll(uint16_t value){
    
}

void Tlc5940::Clear(){
    
}

uint16_t Tlc5940::Get(uint8_t channel){
    
}

int Tlc5940::Update(){
    
}

int Tlc5940::InitializeGpios(){
    int ret = 0;

    gpio0 = device_get_binding("GPIO_0");
	if (gpio0 == NULL) {
		LOG_ERR("***ERROR: GPIO_0 device binding!");
        return -1;
	} 

    /* Configure GSCLK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.gsclk_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 0);
    tlc_cfg.gsclk_gpio.port = gpio0;
    tlc_cfg.gsclk_gpio.pin = 2;
    tlc_cfg.gsclk_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.gsclk_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("GSCLK config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure XLAT pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.xlat_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 1);
    tlc_cfg.xlat_gpio.port = gpio0;
    tlc_cfg.xlat_gpio.pin = 3;
    tlc_cfg.xlat_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.xlat_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("XLAT config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure BLANK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.blank_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 2);
    tlc_cfg.blank_gpio.port = gpio0;
    tlc_cfg.blank_gpio.pin = 7;
    tlc_cfg.blank_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.blank_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("BLANK config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure SIN pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.sin_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 3);
    tlc_cfg.sin_gpio.port = gpio0;
    tlc_cfg.sin_gpio.pin = 8;
    tlc_cfg.sin_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.sin_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("SIN config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure SCLK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.sclk_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 4);
    tlc_cfg.sclk_gpio.port = gpio0;
    tlc_cfg.sclk_gpio.pin = 10;
    tlc_cfg.sclk_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.sclk_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("SCLK config failed, err: %d", ret);
        return -EIO;
    }

    return ret;
}

int Tlc5940::SetPin(uint8_t pin, uint8_t state){
    
    gpio_flags_t extra_flags = state ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE;

    const struct gpio_dt_spec *gpio_spec = NULL;

    switch(pin){
        case GSCLK_PIN:
            gpio_spec = &tlc_cfg.gsclk_gpio;
            break;
        case XLAT_PIN:
            gpio_spec = &tlc_cfg.xlat_gpio;
            break;            
        case BLANK_PIN:
            gpio_spec = &tlc_cfg.blank_gpio;
            break;
        case SIN_PIN:
            gpio_spec = &tlc_cfg.sin_gpio;
            break;            
        case SCLK_PIN:
            gpio_spec = &tlc_cfg.sclk_gpio;
            break;            
        default:
            return -1;
    }

    int ret = gpio_pin_configure_dt(gpio_spec, extra_flags);

    if (ret != 0) {
        LOG_ERR("GPIO config failed, err: %d, extflg: 0x%08X, %s%u", ret, extra_flags,
                gpio_spec->port->name, gpio_spec->pin);
        return -1;
    }

    return ret;

}

bool Tlc5940::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("Tlc5940 BLE Command received");

    Bluetooth::CommandKey bleCommand;
    memcpy(&bleCommand, &key, sizeof(key));
          
    switch(bleCommand.key[0]){
        
        default:
            break;
    }
    
    return true;
}
