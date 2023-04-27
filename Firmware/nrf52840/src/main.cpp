#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <sys/printk.h>

#include "ADS131M08_zephyr.hpp"
#include "max30102.hpp"
#include "mpu6050.hpp"
#include "bme280.hpp"
#include "serial_controller.hpp"
#include "usb_comm_handler.hpp"

#include "ble_service.hpp"

#define ADS_CS              ((uint8_t)15) // 15 (BC840M), 33 (BT840)
#define DATA_READY_GPIO     ((uint8_t)5)  // 5 (BC840M), 6 (BT840)
#define ADS_RESET           ((uint8_t)13)  // 13 (BC840M), 8 (BC840M)

#define ADS_1_CS            ((uint8_t)10) // 10 (BC840M), 36 (BT840)
#define DATA_READY_1_GPIO   ((uint8_t)24)  // 24 (BC840M), 47 (BT840)
#define ADS_1_RESET         ((uint8_t)29) // 29 (BC840M), 35 (BT840)

#define DBG_LED             ((uint8_t)20) // 20 (BC840), 25 (BT840)     
#define MAX_INT             ((uint8_t)28)  // 28 (BC840M), 7 (BT840)
#define MPU_INT             ((uint8_t)2) // 2 (BC840M), 38 (BT840)



// PWM // because we forgot the CLKOUT pin on a draft BT840/BT40 PCB

// #include <drivers/pwm.h>

// #define PWM_CLK     ((uint32_t) 8192000) //Frequency (Hz)
// #define PWM_PERIOD_NSEC ((uint8_t) 122) //1/Frequency in nanosec
// #define PWM_PIN     37 //(BT840 draft 1 missing the CLKOUT pin in same position)

// void initPWM(void) {

//     const struct device *pwm;

//     //ERROR
// 	   pwm = device_get_binding("PWM_0");

//     pwm_pin_set_nsec(
//         pwm, 
//         PWM_PIN, 
//         PWM_PERIOD_NSEC, 
//         PWM_PERIOD_NSEC / 2U, 
//         0
//     );
// }

// //////////////////////////////


static uint8_t LEDn = 0;

static uint32_t LEDt_ms = 100;

static const uint8_t nLEDs = 3;

static bool getAmbient = true;

//list the GPIO in the order we want to flash
static uint8_t LED_gpio[nLEDs] = { 
    22, 25, 255//, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25
};

//LEDs 1.01, 1.11 etc are 32 + the number after the decimal. 1.00 is pin 32 (pretty sure)



LOG_MODULE_REGISTER(main);

/* Static Functions */
static int gpio_init(void);
static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void ads131m08_1_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void interrupt_workQueue_handler(struct k_work* wrk);
static void ads131m08_1_interrupt_workQueue_handler(struct k_work* wrk);
static void max30102_interrupt_workQueue_handler(struct k_work* wrk);
static void mpu6050_interrupt_workQueue_handler(struct k_work* wrk);
static int activate_irq_on_data_ready(void);
static void max30102_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void mpu6050_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

/* Global variables */
const struct device *gpio_0_dev;
const struct device *gpio_1_dev;
struct gpio_callback callback;
struct gpio_callback ads131m08_1_callback;
struct gpio_callback max30102_callback;
struct gpio_callback mpu6050_callback;
struct k_work interrupt_work_item;    ///< interrupt work item
struct k_work ads131m08_1_interrupt_work_item;    ///< interrupt work item
struct k_work max30102_interrupt_work_item;    ///< interrupt work item
struct k_work mpu6050_interrupt_work_item;    ///< interrupt work item
static uint8_t sampleNum = 0;
static uint8_t ads131m08_1_sampleNum = 0;

static uint8_t i = 0;
static uint8_t j = 0;

static uint8_t ble_tx_buff[247] = {0};
static uint8_t ads131m08_1_ble_tx_buff[247] = {0};

// /* size of stack area used by each thread */
#define SSIZE 1024

// /* scheduling priority used by each thread */
#define TPRIORITY 7

static void setupLEDS() {
    for(uint8_t i = 0; i < nLEDs; i++) {
        if(LED_gpio[i] == 255) continue;
        else if(LED_gpio[i] < 32) {
            int ret = gpio_pin_configure(gpio_0_dev, LED_gpio[i], GPIO_OUTPUT_ACTIVE); 
            gpio_pin_set(gpio_0_dev, LED_gpio[i], 0);
        } else {
            int ret = gpio_pin_configure(gpio_1_dev, LED_gpio[i], GPIO_OUTPUT_ACTIVE); 
            gpio_pin_set(gpio_1_dev, LED_gpio[i], 0);
        }
        
    }
}

static void alternateLEDs() {
    if(LED_gpio[LEDn] != 255) {
        if(LED_gpio[LEDn] < 32) {
            gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 0);
        } else {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn], 0);
        }
    }
    
    LEDn++;
    if (LEDn > nLEDs) {
        LEDn = 0;
    }
    
    if(LED_gpio[LEDn] != 255) {
        if(LED_gpio[LEDn] < 32) {
            gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 1);
        } else {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn], 1);
        }
    }
}


void blink(void) {
    setupLEDS();
    while(1) {
	    alternateLEDs();
        k_msleep(LEDt_ms);
    }
}

// does not synchronize correctly
// K_THREAD_DEFINE(blink0_id, SSIZE, blink, NULL, NULL, NULL,
//         TPRIORITY, 0, 0);



//static uint8_t adcRawData[27] = {0};
static max30102_config max30102_default_config = {
    0x80, // Interrupt Config 1. Enable FIFO_A_FULL interrupt
    MAX30102_INTR_2_DIE_TEMP_RDY_EN, // Interrupt Config 2. Enable temperature ready interrupt
    0b01110000, // FIFO Config. Average 16 samples, FIFO Rollover Enabled, FFIO_A_FULL = 32
    0x87, // Mode config. Keep Max30102 shutdown. Multi LED mode .
    0b01110011, // Sp02 config. 800sps rate, 2048 full scale, 18-bit ADC resolution.
    {50, 50}, // LED1/LED2 config. 25.4mA typical LED current
    {0x11, 0x22}  // SLOT config. SLOT1/2 for LED1, SLOT3/4 for LED2.
};

static mpu6050_config mpu6050_default_config = {
    .sample_rate_config = 0x09,     // Sample rate = 100Hz
    .config_reg = 0x01,             // FSYNC disabled. Digital Low Pass filter enabled. 
    .gyro_config = (0x01 << 3),     // 500dps Gyro Full Scale. No Self-Tests.
    .accel_config = (0x01 << 3),    // +/-4g Accel Full Scale. No Self-Tests. 
    .fifo_config = 0x00,            // FIFO disabled.
    .interrupt_pin_config = 0xC0,    // INT active low. Open drain. Keep interrupt pin active until interrupt is cleared. Clear interrupt only by reading INT_STATUS register.
    .interrupt_config = 0x01,       // Enable only Data Ready interrupts.
    .user_control = 0x00,           // Disable FIFO
    .pwr_mgmt_1 = 0x01,             // Use PLL with X axis gyroscope as Clock Source.
    .pwr_mgmt_2 = 0x00              // Don't use Accelerometer only Low Power mode. XYZ axes of Gyro and Accel enabled. 
}; 

ADS131M08 adc;
ADS131M08 adc_1;
SerialController serial;
UsbCommHandler usbCommHandler(serial);
Max30102 max30102(usbCommHandler);
Mpu6050 mpu6050(usbCommHandler);
Bme280 bme280(usbCommHandler);


void main(void)
{
    LOG_ERR("This is a error message!");
    LOG_WRN("This is a warning message!");
    LOG_INF("This is a information message!");
    LOG_DBG("This is a debugging message!");
    // ble_tx_buff[225] = 0x0D;
    // ble_tx_buff[226] = 0x0A;
    int ret = 0;
    uint16_t reg_value = 0;

    ret = gpio_init();

    setupLEDS();

    k_work_init(&interrupt_work_item, interrupt_workQueue_handler);
    k_work_init(&ads131m08_1_interrupt_work_item, ads131m08_1_interrupt_workQueue_handler);
    k_work_init(&max30102_interrupt_work_item, max30102_interrupt_workQueue_handler);
    k_work_init(&mpu6050_interrupt_work_item, mpu6050_interrupt_workQueue_handler);
    if (ret == 0){
        //LOG_INF("GPIOs Int'd!");        
    }

    serial.Initialize();
    usbCommHandler.Initialize();
    adc.init(ADS_CS, DATA_READY_GPIO, ADS_RESET, 8000000); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus
    adc_1.init(ADS_1_CS, DATA_READY_1_GPIO, ADS_1_RESET, 8000000); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus

    max30102.Initialize();
    if(max30102.IsOnI2cBus()){
        LOG_INF("MAX30102 is on I2C bus!");
        max30102.Configure(max30102_default_config);
        max30102.StartSampling();
    } else {
        LOG_WRN("***WARNING: MAX30102 is not connected or properly initialized!");
    }

    mpu6050.Initialize();
    if(mpu6050.IsOnI2cBus()){
        LOG_INF("MPU6050 is on I2C bus!");
        mpu6050.Configure(mpu6050_default_config);
    } else {
        LOG_WRN("***WARNING: MPU6050 is not connected or properly initialized!");
    }

    bme280.Initialize();
    if(bme280.BmX280IsOnI2cBus()){
        LOG_INF("Start BME280 sampling...");
        bme280.StartSampling();
    } else {
        LOG_WRN("***WARNING: BME280 is not connected or properly initialized!");
    }

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

// Write 0 to RESET bit
    if(adc.writeReg(ADS131_MODE,0x0110)){  
        //LOG_INF("ADS131_MODE register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_MODE register.");
    }
    k_msleep(10);
  
//DC Block Filter settings:
    if(adc.writeReg(ADS131_THRSHLD_LSB, 0x0C)){  // Enable DC Block Filter. Write 0x0C to DCBLOCK[3:0] bits. See Table 8-4 in ADS131 datasheet. 
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_THRSHLD_LSB register.");
    }  

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

//ADS131M08_1
    if(adc_1.writeReg(ADS131_CLOCK,0b1111111100011111)){  //< Clock register (page 55 in datasheet)
        //LOG_INF("ADS131_CLOCK register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_1_CLOCK register.");
    }
    k_msleep(10);
    if(adc_1.setGain(32)){    //< Gain Setting, 1-128
        //LOG_INF("ADC Gain properly set to 32");
    } else {
        LOG_ERR("***ERROR: Setting ADC_1 gain!");
    }
    k_msleep(10);
    if(adc_1.writeReg(ADS131_THRSHLD_LSB,0b0000000000001010)){  //< Clock register (page 55 in datasheet)
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
        // adc_1.writeReg(ADS131_CH0_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH1_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH2_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH3_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH4_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH5_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH6_CFG,0b0000000000000000);
        // adc_1.writeReg(ADS131_CH7_CFG,0b0000000000000000);
    } else {
        LOG_ERR("***ERROR: Writing ADS131_1_THRSHLD_LSB register.");
    }
    k_msleep(10);
    
// Write 0 to RESET bit
    if(adc_1.writeReg(ADS131_MODE,0x0110)){  
        //LOG_INF("ADS131_MODE register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_1_MODE register.");
    }
    k_msleep(10);
//DC Block Filter settings:
    if(adc_1.writeReg(ADS131_THRSHLD_LSB,0x0C)){  // Enable DC Block Filter. Write 0x0C to DCBLOCK[3:0] bits. See Table 8-4 in ADS131 datasheet. 
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_1_THRSHLD_LSB register.");
    }  

    reg_value = adc_1.readReg(ADS131_CLOCK);
    //LOG_INF("ADS131_CLOCK: 0x%X", reg_value);
    k_msleep(10); 
    
    reg_value = adc_1.readReg(ADS131_GAIN1);
    //LOG_INF("ADS131_GAIN1: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc_1.readReg(ADS131_ID);
    //LOG_INF("ADS131_ID: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc_1.readReg(ADS131_STATUS);
    //LOG_INF("ADS131_STATUS: 0x%X", reg_value);
    k_msleep(10);

    reg_value = adc_1.readReg(ADS131_MODE);
    //LOG_INF("ADS131_MODE: 0x%X", reg_value);
    k_msleep(10);


    //LOG_INF("Starting in 3...");
    // k_msleep(1000);
    // //LOG_INF("2...");
    // k_msleep(1000);
    // //LOG_INF("1...");
    // k_msleep(1000);

    activate_irq_on_data_ready();

    //uint8_t adcRawData[adc.nWordsInFrame * adc.nBytesInWord] = {0};       

    while(1){

#if 0        
    if(gpio_pin_get(gpio_1_dev, DATA_READY_GPIO)) {           
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
		LOG_ERR("***ERROR: GPIO_0 device binding!");
        return -1;
	}  
    gpio_1_dev = device_get_binding("GPIO_1");
	if (gpio_1_dev == NULL) {
		LOG_ERR("***ERROR: GPIO_1 device binding!");
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
    //activate_irq_on_data_ready();
    //ret = gpio_pin_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_ACTIVE_LOW);
    
    ret = gpio_pin_configure(gpio_0_dev, DBG_LED, GPIO_OUTPUT_ACTIVE); // Set SYNC/RESET pin to HIGH
   
    gpio_pin_set(gpio_0_dev, DBG_LED, 0);
    LOG_INF("Entering sleep...");
    k_sleep(K_MSEC(1000)); // give some time to ADS131 to settle after power on
    LOG_INF("Waking up...");
    gpio_pin_set(gpio_0_dev, DBG_LED, 1);

/* Max30102 Interrupt */
//TODO(bojankoce): Use Zephyr DT (device tree) macros to get GPIO device, port and pin number
    ret += gpio_pin_configure(gpio_0_dev, MAX_INT, GPIO_INPUT | GPIO_PULL_UP); // Pin P0.28
    ret += gpio_pin_interrupt_configure(gpio_0_dev, MAX_INT, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&max30102_callback, max30102_irq_cb, BIT(MAX_INT));    
    ret += gpio_add_callback(gpio_0_dev, &max30102_callback);
    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Max30102 Interrupt pin Int'd!");
    } 

/* MPU6050 Interrupt */
//TODO(bojankoce): Use Zephyr DT (device tree) macros to get GPIO device, port and pin number
    ret += gpio_pin_configure(gpio_0_dev, MPU_INT, GPIO_INPUT | GPIO_PULL_UP); // Pin P0.2
    ret += gpio_pin_interrupt_configure(gpio_0_dev, MPU_INT, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&mpu6050_callback, mpu6050_irq_cb, BIT(MPU_INT));    
    ret += gpio_add_callback(gpio_0_dev, &mpu6050_callback);
    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Mpu6050 Interrupt pin Int'd!");
    } 

    return ret;
}

static int activate_irq_on_data_ready(void){
    int ret = 0;

//ADS131M08_0
    ret += gpio_pin_configure(gpio_1_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += gpio_pin_interrupt_configure(gpio_1_dev, DATA_READY_GPIO, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&callback, ads131m08_drdy_cb, BIT(DATA_READY_GPIO));    
    ret += gpio_add_callback(gpio_1_dev, &callback);
    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Data Ready Int'd!");
    } 

//ADS131M08_1
    ret += gpio_pin_configure(gpio_0_dev, DATA_READY_1_GPIO, GPIO_INPUT | GPIO_PULL_UP);
    ret += gpio_pin_interrupt_configure(gpio_0_dev, DATA_READY_1_GPIO, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&ads131m08_1_callback, ads131m08_1_drdy_cb, BIT(DATA_READY_1_GPIO));    
    ret += gpio_add_callback(gpio_0_dev, &ads131m08_1_callback);
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

static void ads131m08_1_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&ads131m08_1_interrupt_work_item); 
}

static void max30102_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&max30102_interrupt_work_item);     
}

static void mpu6050_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins){
    k_work_submit(&mpu6050_interrupt_work_item);     
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

    ble_tx_buff[225+i] = LED_gpio[LEDn];

    sampleNum++;
    i++;
    if(i == 9){

        alternateLEDs(); //temp, since threads won't synchronize

        i = 0;
        Bluetooth::Ads131m08Notify(ble_tx_buff, 234); //225
        usbCommHandler.SendAds131m08Samples(ble_tx_buff, 234, 0);
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

    ble_tx_buff[226+j] = LED_gpio[LEDn];

    ads131m08_1_sampleNum++;
    //LOG_INF("ADS131M08_1 Sample: %d", ads131m08_1_sampleNum);
    j++;
    if(j == 9){
        j = 0;
        Bluetooth::Ads131m08_1_Notify(ads131m08_1_ble_tx_buff, 234);
        usbCommHandler.SendAds131m08Samples(ads131m08_1_ble_tx_buff, 234, 0);
    }
}

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from MAX30102 interrupt pin 
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
static void max30102_interrupt_workQueue_handler(struct k_work* wrk)
{	
    //LOG_INF("Max30102 Interrupt!");
    max30102.HandleInterrupt();
}

/**
 * @brief IntWorkQueue handler. Used to process interrupts coming from MPU6050 interrupt pin 
 * Because all activity is performed on cooperative level no addition protection against data corruption is required
 * @param wrk work object
 * @warning  Called by system scheduled in cooperative level.
 */
static void mpu6050_interrupt_workQueue_handler(struct k_work* wrk)
{	
    //LOG_INF("MPU6050 Interrupt!");
    mpu6050.HandleInterrupt();
}
