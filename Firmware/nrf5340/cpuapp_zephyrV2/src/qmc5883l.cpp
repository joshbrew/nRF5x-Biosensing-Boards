#include <zephyr.h>
#include <string.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
//#include <sys/__assert.h>
#include <stdlib.h>
#include <logging/log.h>

#include "qmc5883l.hpp"
#include "ble_service.hpp"
#include "usb_comm_handler.hpp"
// For registering callback
#include "ble_service.hpp"

LOG_MODULE_REGISTER(qmc5883l, LOG_LEVEL_INF);

Qmc5883l::Qmc5883l(UsbCommHandler &controller) : serialHandler(controller) {
    LOG_DBG("Qmc5883l Constructor!");
}

int Qmc5883l::Initialize() {

    int ret = 0;
    uint8_t part_id;
    
    LOG_DBG("Starting Qmc5883l Initialization..."); 
    sample_cnt = 0;
    packet_cnt = 0;
    transport.Initialize();   
    
    qmc5883l_is_on_i2c_bus_.store(false, std::memory_order_relaxed);

    part_id = transport.ReadRegister(QMC5883L_WHO_AM_I);

    if(part_id == qmc5883l_id){
        LOG_DBG("SUCCESS: QMC5883L ID match!");
        qmc5883l_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
    } else {
        LOG_ERR("Wrong ID: 0x%X", part_id);
        return -1;
    }
    /* wake up chip */
    //Wakeup();
    Reset();
    k_msleep(5);
//    Shutdown();

    Bluetooth::GattRegisterControlCallback(CommandId::Qmc5883lCmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    return 0;
}

int Qmc5883l::Configure(qmc5883l_config config){
    uint8_t status;   

    transport.WriteRegister(QMC5883L_SET_RESET_PERIOD, 0x01); // Recommended by QMC5883L datasheet
    transport.WriteRegister(QMC5883L_CTRL_REG_1, config.ctrl_reg_1);
    transport.WriteRegister(QMC5883L_CTRL_REG_2, config.ctrl_reg_2);

    // Read one of the XYZ output registers to clear DRDY
    status = transport.ReadRegister(QMC5883L_Z_MSB);

    status = transport.GetStatus();
    return status;
}

bool Qmc5883l::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("Qmc5883l BLE Command received");

    Bluetooth::CommandKey bleCommand;
    memcpy(&bleCommand, &key, sizeof(key));
          
    switch(bleCommand.key[0]){
        case static_cast<uint8_t>(BleCommand::StartSampling):
            StartSampling();
            break;
        case static_cast<uint8_t>(BleCommand::StopSampling):
            StopSampling();
            break;
        
        default:
            break;
    }
    
    return true;
}

void Qmc5883l::Reset() {
    uint8_t user_ctrl_reg;
    // Write 1 to SOFT_RST bit. This will restore default value of all registers.
    // QMC5883L immediately switches to standby mode.
    transport.UpdateRegister(QMC5883L_CTRL_REG_2, BIT(QMC5883L_SOFT_RST_BIT), 0xFF);    

    LOG_DBG("Qmc5883l Reset Success!");
}

void Qmc5883l::Shutdown() {
    // Write 00 to Mode[1:0] bits
    transport.UpdateRegister(QMC5883L_CTRL_REG_1, QMC5833L_MODE_MASK, 0x00);
}

void Qmc5883l::Wakeup() {
    // Write 01 to Mode[1:0] bits
    transport.UpdateRegister(QMC5883L_CTRL_REG_1, QMC5833L_MODE_MASK, 0x01);
}

void Qmc5883l::StartSampling(){    
    Wakeup();
}   

void Qmc5883l::StopSampling(){
    Shutdown();
}

void Qmc5883l::GetDieTemperature(){

}   

void Qmc5883l::HandleInterrupt(){
    //LOG_INF("Handling Qmc5883l interrupt!");
    uint8_t int_reason;
    int_reason = transport.ReadRegister(QMC5883L_STATUS_REG);
    
    if(int_reason & BIT(QMC5883L_OVL_BIT)){
        LOG_DBG("Sensor value overflow!");
        // Read one of the XYZ output registers to clear DRDY
        transport.ReadRegister(QMC5883L_Z_MSB);
    }

    if(int_reason & BIT(QMC5883L_DOR_BIT)){
        LOG_DBG("QMC5883L Data Skip (DOR) Interrupt!");
        // Read one of the XYZ output registers to clear DRDY
        transport.ReadRegister(QMC5883L_Z_MSB);        
    }

    if(int_reason & BIT(QMC5883L_DRDY_BIT)){
        //LOG_INF("QMC5883L Data Ready interrupt!");
        // Store Accel and Gyro samples
        transport.ReadRegisters(QMC5883L_X_LSB, (tx_buf + 6*sample_cnt + 1), 6);        
        sample_cnt++;
        if(sample_cnt == 40){
            //Store Temperature reading
            transport.ReadRegisters(QMC5883L_TOUT_LSB, (tx_buf + 6*sample_cnt + 1), 2);
            tx_buf[0] = packet_cnt;            
            packet_cnt++;
            sample_cnt = 0;          
            Bluetooth::Qmc5883lNotify(tx_buf, 243);
            serialHandler.SendMagnetometerSamples(tx_buf, 243);
        }
    }
} 

bool Qmc5883l::IsOnI2cBus(){
    bool status;
    status = qmc5883l_is_on_i2c_bus_.load(std::memory_order_relaxed);
    return status;
}