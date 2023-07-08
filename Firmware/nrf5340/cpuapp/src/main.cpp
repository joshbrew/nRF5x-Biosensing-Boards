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

#define ADS_CS              ((uint8_t)22)
#define DATA_READY_GPIO     ((uint8_t)15)  
#define ADS_RESET           ((uint8_t)12)  

#define ADS_1_CS            ((uint8_t)30)
#define DATA_READY_1_GPIO   ((uint8_t)11) 
#define ADS_1_RESET         ((uint8_t)9)

#define DBG_LED             ((uint8_t)19) //red LED

#define MAX_INT             ((uint8_t)4)

#define MPU_INT             ((uint8_t)5)

#define PWM_CLK         ((uint32_t)8192000) //Frequency (Hz)
#define PWM_PERIOD_NSEC ((uint8_t)122) //1/Frequency in nanosec
#define PWM_PIN         ((uint8_t)33) //(BT840 draft 1 missing the CLKOUT pin in same position)

bool usePWM = true;

static const uint8_t samplesPerLED = 3;

static bool useLEDS = false;
static uint32_t LEDt_ms = 100;
static const uint8_t nLEDs = 3;


//list the GPIO in the order we want to flash. 255 is ambient
static uint8_t LED_gpio[nLEDs] = { 
    37, 38, 255//, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25, 15, 25, 
    //15, 25
};

//LEDs 1.01, 1.11 etc are 32 + the number after the decimal. 1.00 is pin 32 (pretty sure)

static uint8_t LEDn = 0;
static uint8_t LEDSampleCtr = 0;




LOG_MODULE_REGISTER(main);

/* Static Functions */
static int  gpio_init(void);
static int  sensor_gpio_int(void);
static int  setADS131_int(void);
static void ads131m08_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void ads131m08_1_drdy_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void interrupt_workQueue_handler(struct k_work* wrk);
static void ads131m08_1_interrupt_workQueue_handler(struct k_work* wrk);
static void max30102_interrupt_workQueue_handler(struct k_work* wrk);
static void mpu6050_interrupt_workQueue_handler(struct k_work* wrk);
static void max30102_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static void mpu6050_irq_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

/* Global variables */
const struct device *gpio_0_dev;
const struct device *gpio_1_dev;
struct gpio_callback callback;
struct gpio_callback ads131m08_1_callback;
struct gpio_callback max30102_callback;
struct gpio_callback mpu6050_callback;
struct k_work        interrupt_work_item;    ///< interrupt work item
struct k_work        ads131m08_1_interrupt_work_item;    ///< interrupt work item
struct k_work        max30102_interrupt_work_item;    ///< interrupt work item
struct k_work        mpu6050_interrupt_work_item;    ///< interrupt work item

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

static bool getGPIO(int pin) {
    bool state = false; 
    if(pin < 32) {
        if(gpio_pin_get(gpio_0_dev, pin)) {   
            state = true;
        }
    } else if (pin < 100) {
        if(gpio_pin_get(gpio_1_dev, pin-32)) {   
            state = true;
        }
    } else {
        if(gpio_pin_get(gpio_1_dev, pin-100)) {   
            state = true;
        }
    }
    return state;
} 

static int setGPIO(int pin, int state) {
    int ret = 0;
    if(pin < 32) {
        ret = gpio_pin_set(gpio_0_dev, pin, state);
    } else if (pin < 100) {
        ret = gpio_pin_set(gpio_1_dev, pin-32, state);
    } else {
        ret = gpio_pin_set(gpio_1_dev, pin-100, state);
    }
    return ret;
}

//LED routines using GPIO

static int setupLEDS() {
    int ret = 0;
    for(uint8_t i = 0; i < nLEDs; i++) {
        if(LED_gpio[i] == 255) continue;
        else if(LED_gpio[i] < 32) {
            ret += gpio_pin_configure(gpio_0_dev, LED_gpio[i], GPIO_OUTPUT_ACTIVE); 
            gpio_pin_set(gpio_0_dev, LED_gpio[i], 0);
        } else if (LED_gpio[1] < 100) {
            ret += gpio_pin_configure(gpio_1_dev, LED_gpio[i] - 32, GPIO_OUTPUT_ACTIVE); 
            gpio_pin_set(gpio_1_dev, LED_gpio[i] - 32, 0);
        } else {
            ret += gpio_pin_configure(gpio_1_dev, LED_gpio[i] - 100, GPIO_OUTPUT_ACTIVE);  //supports e.g. 114 for 1.14
            gpio_pin_set(gpio_1_dev, LED_gpio[i] - 100, 0);
        }
        
    }

    return ret;
}


static void alternateLEDs() {
    if(LED_gpio[LEDn] != 255) {
        if(LED_gpio[LEDn] < 32) {
            gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 0);
        } else if (LED_gpio[1] < 100) {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 32, 0);
        } else {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 100, 0);
        }
    }
    
    LEDn++;
    
    if (LEDn >= nLEDs) {
        LEDn = 0;
    }
    
    if(LED_gpio[LEDn] != 255) {
        if(LED_gpio[LEDn] < 32) {
            gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 1);
        } else if (LED_gpio[1] < 100) {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 32, 1);
        } else {
            gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 100, 1);
        }
    }
}

static void incrLEDSampleCtr() {

    LEDSampleCtr++;
    if(LEDSampleCtr >= samplesPerLED) {
        //TODO: implement an LED driver chip. GPIO has bad power up time
        alternateLEDs(); //alternate to the ADC samples to time the LED pulses (for photodiode sampling)
        LEDSampleCtr = 0;
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

    //setADS131_int();
    //ret = gpio_pin_configure(gpio_0_dev, DATA_READY_GPIO, GPIO_INPUT | GPIO_ACTIVE_LOW);
    
    configureGPIO(DBG_LED, GPIO_OUTPUT_ACTIVE); // Set SYNC/RESET pin to HIGH
    setGPIO(DBG_LED, 0);

    LOG_INF("Entering sleep...");
    k_sleep(K_MSEC(1000)); // give some time to ADS131 to settle after power on
    LOG_INF("Waking up...");

    setGPIO(DBG_LED, 1);

}

static int sensor_gpio_int(void){
	int ret = 0;


/* Max30102 Interrupt */
//TODO(bojankoce): Use Zephyr DT (device tree) macros to get GPIO device, port and pin number

    ret += configureGPIO(MAX_INT, GPIO_INPUT | GPIO_PULL_UP);
    ret += configureInterrupt(MAX_INT, GPIO_INT_EDGE_FALLING);
    ret += addGPIOCallback(MAX_INT, &max30102_callback, &max30102_irq_cb);

    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Max30102 Interrupt pin Int'd!");
    } 

/* MPU6050 Interrupt */
//TODO(bojankoce): Use Zephyr DT (device tree) macros to get GPIO device, port and pin number
    ret += configureGPIO(MPU_INT, GPIO_INPUT | GPIO_PULL_UP);
    ret += configureInterrupt(MPU_INT, GPIO_INT_EDGE_FALLING);
    ret += addGPIOCallback(MPU_INT, &mpu6050_callback,&mpu6050_irq_cb);

    if (ret != 0){
        LOG_ERR("***ERROR: GPIO initialization\n");
    } else {
        LOG_INF("Mpu6050 Interrupt pin Int'd!");
    } 

    return ret;
}



static int setADS131_int(void){
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


// PWM // because we forgot the CLKOUT pin on a draft BT840/BT40 PCB

#include <drivers/pwm.h>

void initPWM(void) {

    const struct device *pwm;

	pwm = device_get_binding("PWM_0");

    pwm_pin_set_nsec(
        pwm, 
        PWM_PIN, 
        PWM_PERIOD_NSEC, 
        PWM_PERIOD_NSEC / 2U, 
        0
    );
}

// //////////////////////////////


static int * setupadc(ADS131M08 * adc) {

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


// Write 0 to RESET bit
    if(adc->writeReg(ADS131_MODE,0x0110)){  
        //LOG_INF("ADS131_MODE register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_MODE register.");
    }
    k_msleep(10);
  
//DC Block Filter settings:
    if(
        adc->writeReg(ADS131_THRSHLD_LSB, 0x0C)
    ){  // Enable DC Block Filter. Write 0x0C to DCBLOCK[3:0] bits. See Table 8-4 in ADS131 datasheet. 
        //LOG_INF("ADS131_THRSHLD_LSB register successfully configured");
    } else {
        LOG_ERR("***ERROR: Writing ADS131_THRSHLD_LSB register.");
    }  

    // adc_1.writeReg(ADS131_CH0_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH1_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH2_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH3_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH4_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH5_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH6_CFG,0b0000000000000000);
    // k_msleep(10);
    // adc_1.writeReg(ADS131_CH7_CFG,0b0000000000000000);
    // k_msleep(10);

    int reg_value[5];
    reg_value[0] = adc->readReg(ADS131_CLOCK);
    //LOG_INF("ADS131_CLOCK: 0x%X", reg_value);
    k_msleep(10); 
    
    reg_value[1] = adc->readReg(ADS131_GAIN1);
    //LOG_INF("ADS131_GAIN1: 0x%X", reg_value);
    k_msleep(10);

    reg_value[2] = adc->readReg(ADS131_ID);
    //LOG_INF("ADS131_ID: 0x%X", reg_value);
    k_msleep(10);

    reg_value[3] = adc->readReg(ADS131_STATUS);
    //LOG_INF("ADS131_STATUS: 0x%X", reg_value);
    k_msleep(10);

    reg_value[4] = adc->readReg(ADS131_MODE);
    //LOG_INF("ADS131_MODE: 0x%X", reg_value);
    k_msleep(10);

    return reg_value;
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

    if(useLEDS) incrLEDSampleCtr();

    if(i == 9){
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











/**    MAIN    */

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

    gpio_init();
    ret = sensor_gpio_int();

    if(usePWM) initPWM();
    if(useLEDS) setupLEDS();

    k_work_init(&interrupt_work_item,               interrupt_workQueue_handler);
    k_work_init(&ads131m08_1_interrupt_work_item,   ads131m08_1_interrupt_workQueue_handler);
    k_work_init(&max30102_interrupt_work_item,      max30102_interrupt_workQueue_handler);
    k_work_init(&mpu6050_interrupt_work_item,       mpu6050_interrupt_workQueue_handler);
    
    if (ret == 0){
        //LOG_INF("GPIOs Int'd!");        
    }

    serial.Initialize();
    usbCommHandler.Initialize();
    
    adc.init(
        ADS_CS, 
        DATA_READY_GPIO, 
        ADS_RESET, 
        8192000
    ); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus
    
    adc_1.init(
        ADS_1_CS, 
        DATA_READY_1_GPIO, 
        ADS_1_RESET, 
        8192000
    ); // cs_pin, drdy_pin, sync_rst_pin, 8MHz SPI bus

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

    setupadc(&adc);
    setupadc(&adc_1);
    //LOG_INF("Starting in 3...");
    // k_msleep(1000);
    // //LOG_INF("2...");
    // k_msleep(1000);
    // //LOG_INF("1...");
    // k_msleep(1000);

    setADS131_int();

    //uint8_t adcRawData[adc.nWordsInFrame * adc.nBytesInWord] = {0};       

    while(1){

#if 0        
    bool drdy = getGPIO(DATA_READY_GPIO);

    if(drdy) {
        adc.readAllChannels(adcRawData);

        sampleNum++;
        //LOG_INF("Sample: %d", sampleNum);
        if (sampleNum == 100){
            LOG_INF("ADC[0]: %d", ((adcRawData[3] << 16) | (adcRawData[4] << 8) | adcRawData[5]));
        }
    }      
#endif
    LOG_INF("Hi");
    k_msleep(10000);
    }
}
