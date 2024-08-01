#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/uart.h>
#include <logging/log.h>
#include <drivers/gpio.h>
#include <cstdio>

#include "ADS131M08_zephyr.hpp"

const struct device *uart_dev; /** UART device. For communication with RP2040 */
const struct device *gpio_0_dev;
const struct device *gpio_1_dev;

#define CONFIG_UART_RX_TOUT_US 100000
#define CONFIG_UART_RX_TX_BUF_SZ 32
static uint8_t recvBuffer[CONFIG_UART_RX_TX_BUF_SZ];

LOG_MODULE_REGISTER(main);

#define ADS_CS              ((uint8_t)22)
#define DATA_READY_GPIO     ((uint8_t)15)  
#define ADS_RESET           ((uint8_t)12)  

#define ADS_1_CS            ((uint8_t)30)  // 30 //25 (nirs ensemble)
#define DATA_READY_1_GPIO   ((uint8_t)11)  // 11 //14 (nirs ensemble)
#define ADS_1_RESET         ((uint8_t)9)   // 9  //108 (nirs ensemble)

#define DBG_LED             ((uint8_t)19)  //Red LED

#define MAX_INT             ((uint8_t)4) // 4 //113 (nirs ensemble)

#define MPU_INT             ((uint8_t)5)   // 5 //4 (nirs ensemble)
#define QMC5883L_DRDY       ((uint8_t)6)   // 6 //32 (nirs ensemble)

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{	
    
    switch (evt->type) {
        case UART_TX_DONE:
            LOG_DBG("UART_TX_DONE");             
            break;

        case UART_RX_RDY:
            LOG_DBG("UART_RX_RDY\n");
            LOG_DBG("%d Bytes received from other MCU", evt->data.rx.len);
            LOG_DBG("offset: %d", evt->data.rx.offset);            
            LOG_HEXDUMP_INF(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len, "rp2040_data");         
            break;

        case UART_RX_DISABLED:
            LOG_DBG("UART_RX_DISABLED\n");
            uart_rx_enable(dev, recvBuffer, sizeof(recvBuffer), CONFIG_UART_RX_TOUT_US);
            break;

        case UART_TX_ABORTED:
            LOG_DBG("UART_TX_ABORTED\n");
            break;

        case UART_RX_BUF_REQUEST:
            LOG_DBG("UART_RX_BUF_REQUEST\n");
            break;

        case UART_RX_BUF_RELEASED:
            LOG_DBG("UART_RX_BUF_RELEASED\n");            
            break;

        case UART_RX_STOPPED:
            LOG_DBG("UART_RX_STOPPED\n");
            break;

        default:
            break;
	}
}

#if CONFIG_USE_ADS131M08
/* Static Functions */
static int  init_ads131_gpio_int(void);
static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void ads131m08_1_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void interrupt_workQueue_handler(struct k_work* wrk);
static void ads131m08_1_interrupt_workQueue_handler(struct k_work* wrk);
static int activate_irq_on_data_ready(void);

/* Global variables */
struct gpio_callback callback;
struct gpio_callback ads131m08_1_callback;
struct k_work interrupt_work_item;    ///< interrupt work item
struct k_work ads131m08_1_interrupt_work_item;    ///< interrupt work item

static uint8_t ble_tx_buff[247] = {0};
static uint8_t ads131m08_1_ble_tx_buff[247] = {0};
static uint8_t sampleNum = 0;
static uint8_t ads131m08_1_sampleNum = 0;
static uint8_t i = 0;
static uint8_t j = 0;
#endif 

#if CONFIG_USE_ADS131M08
ADS131M08 adc;
ADS131M08 adc_1;
#endif

static int configureGPIO(int pin, gpio_flags_t rule) {
    int ret = 0;
    if(pin < 32) {
        ret += gpio_pin_configure(gpio_0_dev, pin,       rule); 
    } else if (pin < 100) {
        ret += gpio_pin_configure(gpio_1_dev, pin - 32,  rule); 
    } else {
        ret += gpio_pin_configure(gpio_1_dev, pin - 100, rule); //supports e.g. 114 for 1.14
    }
    return ret;
}

static int configureInterrupt(int pin, gpio_flags_t rule) {
    int ret = 0;
    if(pin < 32) {
        ret += gpio_pin_interrupt_configure(gpio_0_dev, pin, rule);
    } else if (pin < 100) {
        ret += gpio_pin_interrupt_configure(gpio_1_dev, pin - 32,  rule); 
    } else {
        ret += gpio_pin_interrupt_configure(gpio_1_dev, pin - 100, rule); //supports e.g. 114 for 1.14
    }
    return ret;
}

static int addGPIOCallback(int pin, gpio_callback * gpio_cb, void (*cb)(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)) {
    int ret = 0;
    if(pin < 32) {
        gpio_init_callback(gpio_cb, cb, BIT(pin));    
        ret = gpio_add_callback(gpio_0_dev, gpio_cb);
    } else if (pin < 100) {
        gpio_init_callback(gpio_cb, cb, BIT(pin - 32));    
        ret = gpio_add_callback(gpio_1_dev, gpio_cb);
    } else {
        gpio_init_callback(gpio_cb, cb, BIT(pin - 100));    
        ret = gpio_add_callback(gpio_1_dev, gpio_cb); //supports e.g. 114 for 1.14
    }
    return ret;
}

#if CONFIG_USE_ADS131M08
static int init_ads131_gpio_int(void){
    int ret = 0;

//ADS131M08_0
    ret += configureGPIO(DATA_READY_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += configureInterrupt(DATA_READY_GPIO, GPIO_INT_EDGE_FALLING);
    ret += addGPIOCallback(DATA_READY_GPIO, &callback, &ads131m08_drdy_cb);

    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Data Ready Int'd!");
    } 

//ADS131M08_1
    ret += configureGPIO(DATA_READY_1_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += configureInterrupt(DATA_READY_1_GPIO, GPIO_INT_EDGE_FALLING);
    ret += addGPIOCallback(DATA_READY_1_GPIO, &ads131m08_1_callback, &ads131m08_1_drdy_cb);

    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Data Ready 1 Int'd!");
    } 

    return ret;
}

static int setupadc(ADS131M08 * adc) {
    int reg_value = 0;
    #if CONFIG_USE_ADS131M08
    if(adc->writeReg(ADS131_CLOCK,0b1111111100011111)){  //< Clock register (page 55 in datasheet)
        //LOG_INF("ADS131_CLOCK register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_CLOCK register.");
    }
    k_msleep(10);
    if(adc->setGain(32)){    //< Gain Setting, 1-128
        //LOG_INF("ADC Gain properly set to 32");
    } else {
        LOG_ERR("***ERROR: Setting ADC gain!");
    }
    k_msleep(10);
    if(adc->writeReg(ADS131_THRSHLD_LSB,0b0000000000001010)){  //< Clock register (page 55 in datasheet)
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

// Write 0 to RESET bit
    if(adc->writeReg(ADS131_MODE,0x0110)){  
        //LOG_INF("ADS131_MODE register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_MODE register.");
    }
    k_msleep(10);
  
//DC Block Filter settings:
    if(adc->writeReg(ADS131_THRSHLD_LSB,0x04)){  // Enable DC Block Filter. Write 0x04 to DCBLOCK[3:0] bits. See Table 8-4 in ADS131 datasheet. 
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_THRSHLD_LSB register.");
    }  

    reg_value = adc->readReg(ADS131_CLOCK);
    LOG_INF("ADS131_CLOCK: 0x%X", reg_value);
    k_msleep(10); 
    
    reg_value = adc->readReg(ADS131_GAIN1);
    LOG_INF("ADS131_GAIN1: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc->readReg(ADS131_ID);
    LOG_INF("ADS131_ID: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc->readReg(ADS131_STATUS);
    LOG_INF("ADS131_STATUS: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc->readReg(ADS131_MODE);
    LOG_INF("ADS131_MODE: 0x%X", reg_value);
    k_msleep(10);

    #endif
    return reg_value;
}

static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&interrupt_work_item); 
}

static void ads131m08_1_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&ads131m08_1_interrupt_work_item); 
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
    // LOG_INF("ADS131M08 Sample: %d", sampleNum);

    i++;
    if(i == 9){
        i = 0;

        LOG_HEXDUMP_INF(ble_tx_buff, 227, "ADS131m08");
        // LOG_INF("%d", sampleNum);
        // Bluetooth::Ads131m08Notify(ble_tx_buff, 227);
#if CONFIG_USE_USB
        usbCommHandler.SendAds131m08Samples(ble_tx_buff, 227, 0);
#endif        
    }
}

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from ADS131M08_1 Data Ready interrupt pin 
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
static void ads131m08_1_interrupt_workQueue_handler(struct k_work* wrk)
{	
    uint8_t adcBuffer[(adc_1.nWordsInFrame * adc_1.nBytesInWord)] = {0};
    adc_1.readAllChannels(adcBuffer);
    
    ads131m08_1_ble_tx_buff[25*j + 24] = ads131m08_1_sampleNum;
    memcpy((ads131m08_1_ble_tx_buff + 25*j), (adcBuffer + 3), 24);

    ads131m08_1_sampleNum++;
    LOG_INF("ADS131M08_1 Sample: %d", ads131m08_1_sampleNum);
    j++;
    if(j == 9){
        j = 0;
        // Bluetooth::Ads131m08_1_Notify(ads131m08_1_ble_tx_buff, 227);
#if CONFIG_USE_USB
        usbCommHandler.SendAds131m08Samples(ads131m08_1_ble_tx_buff, 227, 0);
#endif        
    }
}
#endif

int init_uart()
{
    int ret = 0;
    uart_dev = device_get_binding(DT_LABEL(DT_NODELABEL(uart3)));
    if (uart_dev == NULL)
    {
            printk("Could not find  %s!\n\r", DT_LABEL(DT_NODELABEL(uart3)));
            return -1;
    }

    ret = uart_callback_set(uart_dev, &uart_cb, NULL);
    if (ret != 0)
    {
            LOG_ERR("uart_callback_set: %d", ret);
            return ret;
    }

    ret = uart_rx_enable(uart_dev, recvBuffer, sizeof(recvBuffer), CONFIG_UART_RX_TOUT_US);
    if (ret != 0)
    {
            LOG_ERR("uart_rx_enable");
            return ret;
    }

    return ret;
}
static int gpio_init(void) {
    
    gpio_0_dev = device_get_binding("GPIO_0");
	if (gpio_0_dev == NULL) {
		LOG_ERR("***ERROR: GPIO_0 device binding!");
        return -1;
	}  
    gpio_1_dev = device_get_binding("GPIO_1");
	if (gpio_1_dev == NULL) {
		LOG_ERR("***ERROR: GPIO_1 device binding!");
        return -1;
	}        

    return 0;
}

static void setupPeripherals() 
{
    gpio_init();

    #if CONFIG_USE_ADS131M08    
        k_work_init(&interrupt_work_item, interrupt_workQueue_handler);
        k_work_init(&ads131m08_1_interrupt_work_item, ads131m08_1_interrupt_workQueue_handler);
    #endif

    #if CONFIG_USE_MCU2MCU
        init_uart();
    #endif

    #if CONFIG_USE_ADS131M08   
        adc.init(ADS_CS, DATA_READY_GPIO, ADS_RESET, 8000000); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus
        // adc_1.init(ADS_1_CS, DATA_READY_1_GPIO, ADS_1_RESET, 8000000); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus

        setupadc(&adc);
        // setupadc(&adc_1);
        init_ads131_gpio_int();
    #endif

    //need to time this correctly with the ADC if controlling LEDs on second MCU
    #if CONFIG_USE_MCU2MCU    
        char cmd_buf[] = "ledr1\r\n";//"debug"; "ledr1"; //"ledr2";
        uart_tx(uart_dev, (uint8_t *)&cmd_buf[0], sizeof(cmd_buf), SYS_FOREVER_MS);
    #endif 


    // uint8_t adcRawData[adc.nWordsInFrame * adc.nBytesInWord] = {0};      
    LOG_INF("Peripherals Setup Done");

}

// Function to configure ADS131_CLOCK register with desired SPS
int configureSPS(ADS131M08* adc, uint16_t osr) {
    // Build the configuration value for the ADS131_CLOCK register
    uint16_t configValue = 0b1111111100000011 | (osr<<2);

    // Write the configuration to the ADS131_CLOCK register
    if (adc->writeReg(ADS131_CLOCK, configValue)) {
        //LOG_INF("ADS131_CLOCK register successfully configured");
        return 0;  // Success
    } else {
        LOG_ERR("***ERROR: Writing ADS131_CLOCK register.");
        return 1;  // Error
    }
}

#define SPS_250_OSR 0b111
#define SPS_500_OSR 0b110
#define SPS_1000_OSR 0b101
#define SPS_2000_OSR 0b100

int main(void)
{
        int ret;

        setupPeripherals();

        while (1)
        {      
            configureSPS(&adc, SPS_250_OSR);
            uint8_t command[] = "ledNum=1,  Config=A\r\n";
            uart_tx(uart_dev, command, sizeof(command), SYS_FOREVER_MS);
            k_msleep(2000);

            configureSPS(&adc, SPS_500_OSR);
            sprintf((char*)command, "ledNum=1,  Config=B\r\n");
            uart_tx(uart_dev, command, sizeof(command), SYS_FOREVER_MS);
            k_msleep(2000);

            configureSPS(&adc, SPS_1000_OSR);
            sprintf((char*)command, "ledNum=1,  Config=C\r\n");
            uart_tx(uart_dev, command, sizeof(command), SYS_FOREVER_MS);
            k_msleep(2000);

            configureSPS(&adc, SPS_2000_OSR);
            sprintf((char*)command, "ledNum=1,  Config=D\r\n");
            uart_tx(uart_dev, command, sizeof(command), SYS_FOREVER_MS);
            k_msleep(2000);
        }

        return 0;
}
