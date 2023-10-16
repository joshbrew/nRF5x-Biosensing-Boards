#include <zephyr.h>
#include <string.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
//#include <sys/__assert.h>
#include <stdlib.h>
#include <logging/log.h>

#include "max30102.hpp"
#include "ble_service.hpp"
#include "usb_comm_handler.hpp"

LOG_MODULE_REGISTER(max30102, LOG_LEVEL_INF);

Max30102::Max30102(UsbCommHandler &controller) : serialHandler(controller) {
    LOG_DBG("Max30102 Constructor!");
}

int Max30102::Initialize() {

    int ret = 0;
    uint8_t part_id;
    uint8_t interrupt_status_reg;
    
    LOG_DBG("Starting Max30102 Initialization..."); 
    packet_cnt = 0;
    transport.Initialize();   
    
    max30102_is_on_i2c_bus_.store(false, std::memory_order_relaxed);

    // Read Interrupt Status Register to clear the interrupt, as recommended in datasheet (page 28)
    // https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf
    interrupt_status_reg = transport.ReadRegister(MAX30102_REG_INT_STS1);
    k_msleep(10);

    part_id = transport.ReadRegister(MAX30102_REG_PART_ID);

    if(part_id == max30102_id){
        LOG_INF("SUCCESS: Max30102 ID match!");
        max30102_is_on_i2c_bus_.store(true, std::memory_order_relaxed);
    } else {
        LOG_ERR("Wrong ID: 0x%X", part_id);
        return -1;
    }

    Reset();
    k_msleep(5);
    Shutdown();

    Bluetooth::GattRegisterControlCallback(CommandId::Max30102Cmd,
        [this](const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset)
        {
            return OnBleCommand(buffer, key, length, offset);
        });

    return 0;
}

bool Max30102::OnBleCommand(const uint8_t *buffer, Bluetooth::CommandKey key, Bluetooth::BleLength length, Bluetooth::BleOffset offset){
    if (offset.value != 0 || length.value == 0)
    {
        return false;
    }
    LOG_DBG("Max30102 BLE Command received");

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

int Max30102::Configure(max30102_config config){
    uint8_t status;   
    
    transport.WriteRegister(MAX30102_REG_INT_EN1, config.interrupt_config_1);
    transport.WriteRegister(MAX30102_REG_INT_EN2, config.interrupt_config_2);
    transport.WriteRegister(MAX30102_REG_FIFO_CFG, config.fifo_config);
    transport.WriteRegister(MAX30102_REG_MODE_CFG, config.mode_config);
    transport.WriteRegister(MAX30102_REG_SPO2_CFG, config.spo2_config);
    transport.WriteRegister(MAX30102_REG_LED1_PA, config.led_pa[0]);
    transport.WriteRegister(MAX30102_REG_LED2_PA, config.led_pa[1]);
    transport.WriteRegister(MAX30102_REG_MULTI_LED1, config.slot_config[0]);
    transport.WriteRegister(MAX30102_REG_MULTI_LED2, config.slot_config[1]);

    status = transport.GetStatus();
    return status;
}

void Max30102::Reset() {
    uint8_t mode_cfg_reg;
    // Write 1 to RESET bit
    transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_RESET_MASK, MAX30102_MODE_CFG_RESET_MASK);

    // Wait for Reset sequence to finish. RESET bit will be cleared
    do{
        mode_cfg_reg = transport.ReadRegister(MAX30102_REG_MODE_CFG);
        k_msleep(5);
//TODO(bojankoce): We can get stuck here. Implement some timeout before returning reset error!        
    } while(mode_cfg_reg & MAX30102_MODE_CFG_RESET_MASK);
    
    LOG_INF("Max30102 Reset Success!");
}

void Max30102::Shutdown() {
    // Write 1 to SHDN bit
    transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, MAX30102_MODE_CFG_SHDN_MASK);
}

void Max30102::Wakeup() {
    uint8_t mode_cfg_reg;
    // Write 0 to SHDN bit
    transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, 0);
}

void Max30102::StartSampling(){
    // Write 0 into SHDN bit of Mode Configuration register
    transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, 0x00);
}   

void Max30102::StopSampling(){
    // Write 1 into SHDN bit of Mode Configuration register
    transport.UpdateRegister(MAX30102_REG_MODE_CFG, MAX30102_MODE_CFG_SHDN_MASK, MAX30102_MODE_CFG_SHDN_MASK);
}

void Max30102::InitiateTemperatureReading(){
    transport.WriteRegister(MAX30102_REG_TEMP_CFG, MAX30102_TEMP_CFG_TEMP_EN);
}   

void Max30102::HandleInterrupt(){
    //LOG_INF("Handling Max30102 interrupt!");
    uint8_t int_reason;
    int_reason = transport.ReadRegister(MAX30102_REG_INT_STS1);
    
    if(int_reason & FIFO_A_FULL_MASK){
        transport.ReadRegisters(MAX30102_REG_FIFO_DATA, (tx_buf + 1), 192);
        //LOG_INF("tx_buf: 0x%X 0x%X 0x%X", tx_buf[0], tx_buf[1], tx_buf[2]);
        InitiateTemperatureReading();
    }

    if(int_reason & PPG_RDY_MASK){
        LOG_INF("PPG Ready!");
    }

    if(int_reason & ALC_OVF_MASK){
        LOG_INF("ALC_OVF!");
    }
    
    if(int_reason & PWR_RDY_MASK){
        LOG_INF("Power Ready!");
    }

    int_reason = transport.ReadRegister(MAX30102_REG_INT_STS2);

    if(int_reason & DIE_TEMP_RDY_MASK){
        //LOG_INF("Temperature Ready!");
        TemperatureRead();
        Bluetooth::Max30102Notify(tx_buf, 195);
        serialHandler.SendMax30102Samples(tx_buf, 195);
    }
} 

void Max30102::TemperatureRead(){
    uint8_t tint;
    uint8_t tfrac;
    tint = transport.ReadRegister(MAX30102_REG_TINT);
    tfrac = transport.ReadRegister(MAX30102_REG_TFRAC);
    //LOG_INF("Temperature: %d, %d", tint, tfrac);
    tx_buf[193] = tint;
    tx_buf[194] = tfrac;
    tx_buf[0] = packet_cnt;            
    packet_cnt++;
}

bool Max30102::IsOnI2cBus(){
    bool status;
    status = max30102_is_on_i2c_bus_.load(std::memory_order_relaxed);
    return status;
}