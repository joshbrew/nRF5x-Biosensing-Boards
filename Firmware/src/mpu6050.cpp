#include <zephyr.h>
#include <string.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
//#include <sys/__assert.h>
#include <stdlib.h>
#include <logging/log.h>

#include "mpu6050.hpp"
#include "ble_service.hpp"

LOG_MODULE_REGISTER(mpu6050);

Mpu6050::Mpu6050() {
    LOG_INF("Mpu6050 Constructor!");
}

int Mpu6050::Initialize() {

    int ret = 0;
    uint8_t part_id;
    
    LOG_INF("Starting Mpu6050 Initialization..."); 
    transport.Initialize();   
    
    mpu6050_is_on_i2c_bus_.store(false, std::memory_order_relaxed);

    part_id = transport.ReadRegister(MPU6050_RA_WHO_AM_I);

    if(part_id == mpu6050_id){
        LOG_INF("SUCCESS: Max30102 ID match!");
        mpu6050_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
    } else {
        LOG_ERR("Wrong ID: 0x%X", part_id);
        return -1;
    }
    /* wake up chip */
    Wakeup();
    Reset();
    k_msleep(5);
//    Shutdown();

    return 0;
}

int Mpu6050::Configure(mpu6050_config config){
    uint8_t status;   
    
    transport.WriteRegister(MPU6050_RA_SMPLRT_DIV, config.sample_rate_config);
    transport.WriteRegister(MPU6050_RA_CONFIG, config.config_reg);
    transport.WriteRegister(MPU6050_RA_GYRO_CONFIG, config.gyro_config);
    transport.WriteRegister(MPU6050_RA_ACCEL_CONFIG, config.accel_config);
    transport.WriteRegister(MPU6050_RA_FIFO_EN, config.fifo_config);
    transport.WriteRegister(MPU6050_RA_INT_PIN_CFG, config.interrupt_pin_config);
    transport.WriteRegister(MPU6050_RA_INT_ENABLE, config.interrupt_config);
    transport.WriteRegister(MPU6050_RA_USER_CTRL, config.user_control);
    transport.WriteRegister(MPU6050_RA_PWR_MGMT_1, config.pwr_mgmt_1);
    transport.WriteRegister(MPU6050_RA_PWR_MGMT_2, config.pwr_mgmt_2);

    status = transport.GetStatus();
    return status;
}

void Mpu6050::Reset() {
    uint8_t user_ctrl_reg;
    // Write 1 to SIG_COND_RESET bit. This will reset signal paths for all sensors and also clear the sensor registers.
    transport.UpdateRegister(MPU6050_RA_USER_CTRL, BIT(MPU6050_USERCTRL_SIG_COND_RESET_BIT), 0xFF);    
    // Wait for Reset sequence to finish. SIG_COND_RESET bit will be cleared
    do{
        user_ctrl_reg = transport.ReadRegister(MPU6050_RA_USER_CTRL);
        k_msleep(5);
//TODO(bojankoce): We can get stuck here. Implement some timeout before returning reset error!        
    } while(user_ctrl_reg & BIT(MPU6050_USERCTRL_SIG_COND_RESET_BIT));

    //transport.WriteRegister(MPU6050_RA_SIGNAL_PATH_RESET, 0x07); // GYRO_RESET, ACCEL_RESET, TEMP_RESET

    LOG_INF("Mpu6050 Reset Success!");
}

void Mpu6050::Shutdown() {
    // Write 1 to SHDN bit
    transport.UpdateRegister(MPU6050_RA_PWR_MGMT_1, BIT(MPU6050_PWR1_SLEEP_BIT), 0xFF);
}

void Mpu6050::Wakeup() {
    // Write 0 to SHDN bit
    transport.UpdateRegister(MPU6050_RA_PWR_MGMT_1, BIT(MPU6050_PWR1_SLEEP_BIT), 0);
}

void Mpu6050::StartSampling(){
    // Write 0 into SHDN bit of Mode Configuration register
    //transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, 0x00);
}   

void Mpu6050::StopSampling(){
    // Write 1 into SHDN bit of Mode Configuration register
    //transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, MAX30102_MODE_CFG_SHDN_MASK);
}

void Mpu6050::GetDieTemperature(){
    //transport.WriteRegister(MAX30102_REG_TEMP_CFG, MAX30102_TEMP_CFG_TEMP_EN);
}   

void Mpu6050::HandleInterrupt(){
    //LOG_INF("Handling Max30102 interrupt!");
    uint8_t int_reason;
    int_reason = transport.ReadRegister(MPU6050_RA_INT_STATUS);
    
    if(int_reason & BIT(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)){
        //transport.ReadRegisters(MAX30102_REG_FIFO_DATA, tx_buf, 192);        
        //InitiateTemperatureReading();
        LOG_INF("FIFO overflow!");
    }

    if(int_reason & BIT(MPU6050_INTERRUPT_I2C_MST_INT_BIT)){
        LOG_INF("I2C master interrupt!");
    }

    if(int_reason & BIT(MPU6050_INTERRUPT_DATA_RDY_BIT)){
        LOG_INF("Data Ready interrupt!");
    }
    
    //if(int_reason & DIE_TEMP_RDY_MASK){
        //LOG_INF("Temperature Ready!");
    //    TemperatureRead();
    //    Bluetooth::Max30102Notify(tx_buf, 194);
    //}
} 

void Mpu6050::TemperatureRead(){

}

bool Mpu6050::IsOnI2cBus(){
    bool status;
    status = mpu6050_is_on_i2c_bus_.load(std::memory_order_relaxed);
    return status;
}