// #pragma once
// // PWM // because we forgot the CLKOUT pin on a draft BT840/BT40 PCB
// #include <drivers/pwm.h>
// #include <drivers/gpio.h>

// #include "gpio_macros.cpp"

// // /* size of stack area used by each thread */
// #define SSIZE 1024

// // /* scheduling priority used by each thread */
// #define TPRIORITY 7


// #define PWM_CLK         ((uint32_t)8192000) //Frequency (Hz)
// #define PWM_PERIOD_NSEC ((uint8_t)122) //1/Frequency in nanosec
// #define PWM_PIN         ((uint8_t)33) //set in overlay //(BT840 draft 1 missing the CLKOUT pin in same position)

// static bool usePWM = false; //fix for a prototype not having a CLKOUT pin proper

// static const uint8_t samplesPerLED = 3;
// static const uint8_t samplesPerAmbient = 3;

// //LEDs 1.01, 1.11 etc are 32 + the number after the decimal. 1.00 is pin 32 (pretty sure)

// static uint8_t LEDn = 0;
// static uint8_t LEDSampleCtr = 0;
// //static uint32_t LEDt_ms = 100; //for a blink routine that doesnt work with all the threads



// #if DT_NODE_HAS_STATUS(DT_ALIAS(pwmadc), okay)
// #define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwmadc))
// void initPWM(void) {

//     const struct device *pwm;

// 	pwm = device_get_binding("PWM_0");

//     pwm_pin_set_nsec(
//         pwm, 
//         PWM_CHANNEL, 
//         PWM_PERIOD_NSEC, 
//         PWM_PERIOD_NSEC / 2, 
//         0
//     );
// }
// #endif




// static const uint8_t nLEDs = 4; //4 //7 //19 //3 ///includes ambient reading (255)
// //list the GPIO in the order we want to flash. 255 is ambient
// static uint8_t LED_gpio[nLEDs] = { 
//     //37, 38, 255
    
//     255, //ambient
//     9, 255, 30//, 255, 10 //11, 13, 12 //2 channel hookup
    
//     //16 channel hookup
//     // 10, 109,
//     // 14, 107,
//     // 114, 17,
//     // 100,  6,
//     // 115, 21,
//     // 16,  24,
//     // 5,   31,
//     // 8,    7,
//     // 111, 104
    
// };

// //LED routines using GPIO

// static int setupLEDS() {
//     int ret = 0;
//     for(uint8_t i = 0; i < nLEDs; i++) {
//         if(LED_gpio[i] == 255) continue;
//         else if(LED_gpio[i] < 32) {
//             ret += gpio_pin_configure(gpio_0_dev, LED_gpio[i], GPIO_OUTPUT_ACTIVE); 
//             gpio_pin_set(gpio_0_dev, LED_gpio[i], 0);
//         } else if (LED_gpio[1] < 100) {
//             ret += gpio_pin_configure(gpio_1_dev, LED_gpio[i] - 32, GPIO_OUTPUT_ACTIVE); 
//             gpio_pin_set(gpio_1_dev, LED_gpio[i] - 32, 0);
//         } else {
//             ret += gpio_pin_configure(gpio_1_dev, LED_gpio[i] - 100, GPIO_OUTPUT_ACTIVE);  //supports e.g. 114 for 1.14
//             gpio_pin_set(gpio_1_dev, LED_gpio[i] - 100, 0);
//         }
        
//     }

//     return ret;
// }


// static void alternateLEDs() {
//     if(LED_gpio[LEDn] != 255) {
//         if(LED_gpio[LEDn] < 32) {
//             gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 0);
//         } else if (LED_gpio[1] < 100) {
//             gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 32, 0);
//         } else {
//             gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 100, 0);
//         }
//     }
    
//     LEDn++;
    
//     if (LEDn >= nLEDs) {
//         LEDn = 0;
//     }
    
//     if(LED_gpio[LEDn] != 255) {
//         if(LED_gpio[LEDn] < 32) {
//             gpio_pin_set(gpio_0_dev, LED_gpio[LEDn], 1);
//         } else if (LED_gpio[1] < 100) {
//             gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 32, 1);
//         } else {
//             gpio_pin_set(gpio_1_dev, LED_gpio[LEDn] - 100, 1);
//         }
//     }
// }

// static void incrLEDSampleCtr() {

//     LEDSampleCtr++;
//     if(LED_gpio[LEDn] != 255 && LEDSampleCtr >= samplesPerLED) {
//         //TODO: implement an LED driver chip. GPIO has bad power up time
//         alternateLEDs(); //alternate to the ADC samples to time the LED pulses (for photodiode sampling)
//         LEDSampleCtr = 0;
//     } else if(LEDSampleCtr >= samplesPerAmbient) {
//         alternateLEDs();
//         LEDSampleCtr = 0;
//     }
// }

// // void blink(void) {
// //     setupLEDS();
// //     while(1) {
// // 	    alternateLEDs();
// //         k_msleep(LEDt_ms);
// //     }
// // }

// // does not synchronize correctly
// // K_THREAD_DEFINE(blink0_id, SSIZE, blink, NULL, NULL, NULL,
// //         TPRIORITY, 0, 0);
