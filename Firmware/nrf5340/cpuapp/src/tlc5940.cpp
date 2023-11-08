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

volatile bool update_in_progress;

/* Work handler function */
void gsUpdate_work_handler(struct k_work *work) {
    update_in_progress = false;   
}
/* Register the work handler */
K_WORK_DEFINE(gsUpdate_work, gsUpdate_work_handler);

namespace
{
    constexpr static int stackSizeTlc = 2048;           ///< Worker thread size
    constexpr static int taskPriorityTlc = 1;           ///< Worker thread priority
    K_THREAD_STACK_DEFINE(pollStackAreaTlc, stackSizeTlc); ///< Worker thread stack
} // namespace

Tlc5940::Tlc5940() {
    LOG_DBG("Tlc5940 Constructor!");
}

int Tlc5940::Initialize(uint16_t initialValue) {

    int ret = 0;
        
    LOG_DBG("Starting Tlc5940 Initialization..."); 
    
    InitializeGpios();
    numChannels = 16 * CONFIG_NUM_TLCS;
    firstCycleFlag = 0;
    
    p_gsData = static_cast<uint8_t *>(&gsData[0]);
    
    SetAll(initialValue);

    LOG_DBG("Tlc5940 Intl'ed!");

    Bluetooth::GattRegisterControlCallback(CommandId::Tlc5940Cmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    // Start working thread
    k_thread_create(&worker, pollStackAreaTlc, K_THREAD_STACK_SIZEOF(pollStackAreaTlc),
                    &TlcWorkingThread, this, nullptr, nullptr, taskPriorityTlc, 0, K_NO_WAIT);

    return ret;
}

int Tlc5940::Set(uint8_t channel, uint16_t value){

    if(channel > numChannels){
        return -1;
    }

    uint16_t index8 = (CONFIG_NUM_TLCS * 16 - 1) - channel;
    uint8_t *index12p = p_gsData + ((((uint16_t)index8) * 3) >> 1);

    if (index8 & 1) { // starts in the middle
                      // first 4 bits intact | 4 top bits of value
        *index12p = (*index12p & 0xF0) | (value >> 8);
                      // 8 lower bits of value
        *(++index12p) = value & 0xFF;
    } else { // starts clean
                      // 8 upper bits of value
        *(index12p++) = value >> 4;
                      // 4 lower bits of value | last 4 bits intact
        *index12p = ((uint8_t)(value << 4)) | (*index12p & 0xF);
    }

    return 0;
}

void Tlc5940::SetAll(uint16_t value){

    for (int i = 0; i < numChannels; i++){
        Set(i, value);
    }
}

void Tlc5940::Clear(){
    memset(p_gsData, 0, sizeof(gsData));
}

uint16_t Tlc5940::Get(uint8_t channel){

    uint16_t value = 0;

    uint16_t index8 = (CONFIG_NUM_TLCS * 16 - 1) - channel;
    uint8_t *index12p = p_gsData + ((((uint16_t)index8) * 3) >> 1);
    return (index8 & 1)? // starts in the middle
            (((uint16_t)(*index12p & 15)) << 8) | // upper 4 bits
            *(index12p + 1)                       // lower 8 bits
        : // starts clean
            (((uint16_t)(*index12p)) << 4) | // upper 8 bits
            ((*(index12p + 1) & 0xF0) >> 4); // lower 4 bits

    return value;
}

int Tlc5940::Update(){
    bool first_time_here = true;

    //TODO: read VCPRG state. If it is HIGH
    // set it to LOW and set firstCycleFlag flag to 1
    // On the Sparkfun TLC5940 devkit, VCPRG is tied to GND
    uint8_t ByteCounter = 0;
    uint8_t GSCLK_Counter = 0;

    update_in_progress = true;
    
    for(int i = 0; i < CONFIG_NUM_TLCS * 24; i++){
        ShiftByte(*(p_gsData + i)); 
    }

    if (firstCycleFlag) {
        PulsePin(SCLK_PIN, LOW_TO_HIGH);
    } else {
        firstCycleFlag = 1;
    }  

    SetPin(BLANK_PIN, 1);
    PulsePin(XLAT_PIN, LOW_TO_HIGH);
    SetPin(BLANK_PIN, 0);

    for(int j = 0; j < 4096; j++){
        PulsePin(GSCLK_PIN, LOW_TO_HIGH);
    }

    return 0;
}

int Tlc5940::InitializeGpios(){
    int ret = 0;

    gpio_dev = device_get_binding("GPIO_1");
	if (gpio_dev == NULL) {
		LOG_ERR("***ERROR: GPIO_0 device binding!");
        return -1;
	} 

    /* Configure GSCLK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.gsclk_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 0);
    
    tlc_cfg.gsclk_gpio.port = gpio_dev;
    tlc_cfg.gsclk_gpio.pin = 7;
    tlc_cfg.gsclk_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.gsclk_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("GSCLK config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure XLAT pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.xlat_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 1);
    tlc_cfg.xlat_gpio.port = gpio_dev;
    tlc_cfg.xlat_gpio.pin = 8;
    tlc_cfg.xlat_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.xlat_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("XLAT config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure BLANK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.blank_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 2);
    tlc_cfg.blank_gpio.port = gpio_dev;
    tlc_cfg.blank_gpio.pin = 4;
    tlc_cfg.blank_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.blank_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("BLANK config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure SIN pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.sin_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 3);
    tlc_cfg.sin_gpio.port = gpio_dev;
    tlc_cfg.sin_gpio.pin = 5;
    tlc_cfg.sin_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.sin_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("SIN config failed, err: %d", ret);
        return -EIO;
    }

    /* Configure SCLK pin to active */
    //TODO: Get the GPIO details from the device tree (.overlay file). See tlc5940 node
    // tlc_cfg.sclk_gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tlc5940), tlc_gpios, 4);
    tlc_cfg.sclk_gpio.port = gpio_dev;
    tlc_cfg.sclk_gpio.pin = 6;
    tlc_cfg.sclk_gpio.dt_flags = GPIO_ACTIVE_HIGH;

    ret = gpio_pin_configure_dt(&tlc_cfg.sclk_gpio, GPIO_OUTPUT_INACTIVE);
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

int Tlc5940::PulsePin(uint8_t pin, uint8_t pulse_dir){
    if (pulse_dir == LOW_TO_HIGH){
            SetPin(pin, 1);
            SetPin(pin, 0);
    } else {
            SetPin(pin, 0);
            SetPin(pin, 1);
    }
    return 0;
}

inline void Tlc5940::ShiftByte(uint8_t byte){
    
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
        if (bit & byte) {
            SetPin(SIN_PIN, 1);
        } else {
            SetPin(SIN_PIN, 0);
        }
        PulsePin(SCLK_PIN, LOW_TO_HIGH);
    }
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
