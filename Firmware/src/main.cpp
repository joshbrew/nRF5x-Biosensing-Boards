#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/printk.h>

#include "ADS131M08_zephyr.hpp"

#include "ble_service.hpp"

// Bluetooth part
//#include <bluetooth/bluetooth.h>
//#include <bluetooth/hci.h>
//#include <bluetooth/conn.h>
//#include <bluetooth/uuid.h>
//#include <bluetooth/gatt.h>

#define DATA_READY_GPIO     ((uint8_t)4)
#define DBG_LED             ((uint8_t)13)

LOG_MODULE_REGISTER(main);

/* Static Functions */
static int gpio_init(void);
static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void interrupt_workQueue_handler(struct k_work* wrk);
static int activate_irq_on_data_ready(void);


/* Global variables */
const struct device *gpio_0_dev;
struct gpio_callback callback;
struct k_work interrupt_work_item;    ///< interrupt work item
static uint8_t sampleNum = 0;
static uint8_t i = 0;

static uint8_t ble_tx_buff[247] = {0};
//static uint8_t adcRawData[27] = {0};

ADS131M08 adc;

void main(void)
{
    LOG_ERR("This is a error message!");
    LOG_WRN("This is a warning message!");
    LOG_INF("This is a information message!");
    LOG_DBG("This is a debugging message!");
    ble_tx_buff[225] = 0x0D;
    ble_tx_buff[226] = 0x0A;
    int ret = 0;
    uint16_t reg_value = 0;

    ret = gpio_init();
    k_work_init(&interrupt_work_item, interrupt_workQueue_handler);
    if (ret == 0){
        //LOG_INF("GPIOs Int'd!");        
    }

    adc.init(5, 4, 7, 8000000); // cs_pin, drdy_pin, sync_rst_pin, 2MHz SPI bus

    Bluetooth::SetupBLE();

    if(adc.writeReg(ADS131_CLOCK,0b1111111100011111)){  //< Clock register (page 55 in datasheet)
        //LOG_INF("ADS131_CLOCK register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_CLOCK register.");
    }
    k_msleep(10);
    if(adc.setGain(32)){    //< Gain Setting, 1-128
        //LOG_INF("ADC Gain properly set to 32");
    } else {
        LOG_ERR("***ERROR: Setting ADC gain!");
    }
    k_msleep(10);
    if(adc.writeReg(ADS131_THRSHLD_LSB,0b0000000000001010)){  //< Clock register (page 55 in datasheet)
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
        // adc.writeReg(ADS131_CH0_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH1_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH2_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH3_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH4_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH5_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH6_CFG,0b0000000000000000);
        // adc.writeReg(ADS131_CH7_CFG,0b0000000000000000);
    } else {
        LOG_ERR("***ERROR: Writing ADS131_THRSHLD_LSB register.");
    }
    k_msleep(10);
    reg_value = adc.readReg(ADS131_CLOCK);
    //LOG_INF("ADS131_CLOCK: 0x%X", reg_value);
    k_msleep(10); 
    
    reg_value = adc.readReg(ADS131_GAIN1);
    //LOG_INF("ADS131_GAIN1: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc.readReg(ADS131_ID);
    //LOG_INF("ADS131_ID: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc.readReg(ADS131_STATUS);
    //LOG_INF("ADS131_STATUS: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc.readReg(ADS131_MODE);
    //LOG_INF("ADS131_MODE: 0x%X", reg_value);
    k_msleep(10);
    
    //LOG_INF("Starting in 3...");
    k_msleep(1000);
    //LOG_INF("2...");
    k_msleep(1000);
    //LOG_INF("1...");
    k_msleep(1000);

    activate_irq_on_data_ready();

    //uint8_t adcRawData[adc.nWordsInFrame * adc.nBytesInWord] = {0};       

    while(1){

#if 0        
        if(gpio_pin_get(gpio_0_dev, DATA_READY_GPIO)) {           
            adc.readAllChannels(adcRawData);

            sampleNum++;
            //LOG_INF("Sample: %d", sampleNum);
            if (sampleNum == 100){
                LOG_INF("ADC[0]: %d", ((adcRawData[3] << 16) | (adcRawData[4] << 8) | adcRawData[5]));
            }
        }
        else {
        }        
#endif
    LOG_INF("Hi");
    k_msleep(10000);
    }
}

static int gpio_init(void){
	int ret = 0;

    gpio_0_dev = device_get_binding("GPIO_0");
	if (gpio_0_dev == NULL) {
		LOG_ERR("***ERROR: GPIO device binding!");
        return -1;
	}    
#if 0
    ret += gpio_pin_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += gpio_pin_interrupt_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&callback, ads131m08_drdy_cb, BIT(DATA_READY_GPIO));    
    ret += gpio_add_callback(gpio_0_dev, &callback);
    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    }
#endif

    //ret = gpio_pin_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_ACTIVE_LOW);
    
    ret = gpio_pin_configure(gpio_0_dev, DBG_LED, GPIO_OUTPUT_ACTIVE); // Set SYNC/RESET pin to HIGH
   
    gpio_pin_set(gpio_0_dev, DBG_LED, 0);
    LOG_INF("Entering sleep...");
    k_sleep(K_MSEC(1000)); // give some time to ADS131 to settle after power on
    LOG_INF("Waking up...");
    gpio_pin_set(gpio_0_dev, DBG_LED, 1);


    return ret;
}

static int activate_irq_on_data_ready(void){
    int ret = 0;

    ret += gpio_pin_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += gpio_pin_interrupt_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&callback, ads131m08_drdy_cb, BIT(DATA_READY_GPIO));    
    ret += gpio_add_callback(gpio_0_dev, &callback);
    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Data Ready Int'd!");
    } 

    return ret;
}

static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&interrupt_work_item); 
}

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from ADS131M08 Data Ready interrupt pin 
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
static void interrupt_workQueue_handler(struct k_work* wrk)
{	
    uint8_t adcBuffer[(adc.nWordsInFrame * adc.nBytesInWord)] = {0};
    adc.readAllChannels(adcBuffer);
    
    ble_tx_buff[25*i + 24] = sampleNum;
    memcpy((ble_tx_buff + 25*i), (adcBuffer + 3), 24);

    sampleNum++;
    i++;
    if(i == 9){
        i = 0;
        Bluetooth::SensorNotify(ble_tx_buff, 227);
    }

}