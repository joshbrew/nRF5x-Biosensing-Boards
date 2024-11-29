
// #pragma once
// #include <drivers/gpio.h>
// #include "gpio_devices.cpp"

// static int configureGPIO(int pin, gpio_flags_t rule) {
//     int ret = 0;
//     if(pin < 32) {
//         ret += gpio_pin_configure(gpio_0_dev, pin,       rule); 
//     } else if (pin < 100) {
//         ret += gpio_pin_configure(gpio_1_dev, pin - 32,  rule); 
//     } else {
//         ret += gpio_pin_configure(gpio_1_dev, pin - 100, rule); //supports e.g. 114 for 1.14
//     }
//     return ret;
// }

// static int configureInterrupt(int pin, gpio_flags_t rule) {
//     int ret = 0;
//     if(pin < 32) {
//         ret += gpio_pin_interrupt_configure(gpio_0_dev, pin, rule);
//     } else if (pin < 100) {
//         ret += gpio_pin_interrupt_configure(gpio_1_dev, pin - 32,  rule); 
//     } else {
//         ret += gpio_pin_interrupt_configure(gpio_1_dev, pin - 100, rule); //supports e.g. 114 for 1.14
//     }
//     return ret;
// }

// static int addGPIOCallback(int pin, gpio_callback * gpio_cb, void (*cb)(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)) {
//     int ret = 0;
//     if(pin < 32) {
//         gpio_init_callback(gpio_cb, cb, BIT(pin));    
//         ret = gpio_add_callback(gpio_0_dev, gpio_cb);
//     } else if (pin < 100) {
//         gpio_init_callback(gpio_cb, cb, BIT(pin - 32));    
//         ret = gpio_add_callback(gpio_1_dev, gpio_cb);
//     } else {
//         gpio_init_callback(gpio_cb, cb, BIT(pin - 100));    
//         ret = gpio_add_callback(gpio_1_dev, gpio_cb); //supports e.g. 114 for 1.14
//     }
//     return ret;
// }

// static bool getGPIO(int pin) {
//     bool state = false; 
//     if(pin < 32) {
//         if(gpio_pin_get(gpio_0_dev, pin)) {   
//             state = true;
//         }
//     } else if (pin < 100) {
//         if(gpio_pin_get(gpio_1_dev, pin-32)) {   
//             state = true;
//         }
//     } else {
//         if(gpio_pin_get(gpio_1_dev, pin-100)) {   
//             state = true;
//         }
//     }
//     return state;
// } 

// static int setGPIO(int pin, int state) {
//     int ret = 0;
//     if(pin < 32) {
//         ret = gpio_pin_set(gpio_0_dev, pin, state);
//     } else if (pin < 100) {
//         ret = gpio_pin_set(gpio_1_dev, pin-32, state);
//     } else {
//         ret = gpio_pin_set(gpio_1_dev, pin-100, state);
//     }
//     return ret;
// }

