#include "ADS131M08.h"

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <stdlib.h>
#include <drivers/spi.h>
#include <logging/log.h>

ADS131M08::ADS131M08(int cs, int xtal, int drdy, int clk) {

    CS = cs; XTAL = xtal; DRDY = drdy; //You don't have to use DRDY, can also read off the ADS131_STATUS register.
    SpiClk = clk;
}

void ADS131M08::init(int clkin) {

    Serial.println("Setting pin configuration");
        
    pinMode(CS, OUTPUT); digitalWrite(CS, HIGH);
    pinMode(DRDY, INPUT_PULLUP);
    
    spi->begin();

    Serial.println("Setting oscillator");

    ledcSetup(2, clkin, 2);
    ledcAttachPin(XTAL, 2); //Simulate 8.192Mhz crystal with PWM. This needs to be started as soon as possible after powering on
    ledcWrite(2,2);

    Serial.println("SPI Ready...");

}

void ADS131M08::readChannels(int8_t * channelArrPtr, int8_t channelArrLen, int32_t * outputArrPtr) {
    
    uint32_t rawDataArr[10];

    // Get data
    spiCommFrame(&rawDataArr[0]);
    
    // Save the decoded data for each of the channels
    for (int8_t i = 0; i<channelArrLen; i++) {
        *outputArrPtr = twoCompDeco(rawDataArr[*channelArrPtr+1]);
        outputArrPtr++;
        channelArrPtr++;
    }
    
}

void ADS131M08::readAllChannels(int32_t inputArr[8]) {
    uint32_t rawDataArr[10];
    int8_t channelArrPtr = 0;
    int8_t channelArrLen = 8;

    // Get data
    spiCommFrame(&rawDataArr[0]);
    // Save the decoded data for each of the channels
    for (int8_t i = 0; i<8; i++) {
        inputArr[i] = twoCompDeco(rawDataArr[channelArrPtr+1]);
        channelArrPtr++;
    }
}

int32_t ADS131M08::readChannelSingle(int8_t channel) {
    /* Returns raw value from a single channel
        channel input from 0-7
    */
    
    int32_t outputArr[1];
    int8_t channelArr[1] = {channel};

    readChannels(&channelArr[0], 1, &outputArr[0]);

    return outputArr[0];
}

bool ADS131M08::globalChop(bool enabled, uint8_t log2delay) {
    /* Function to configure global chop mode for the ADS131M04.

        INPUTS:
        enabled - Whether to enable global-chop mode.
        log2delay   - Base 2 log of the desired delay in modulator clocks periods
        before measurment begins
        Possible values are between and including 1 and 16, to give delays
        between 2 and 65536 clock periods respectively
        For more information, refer to the datasheet.

        Returns true if settings were written succesfully.
    */

    uint8_t delayRegData = log2delay - 1;

    // Get current settings for current detect mode from the CFG register
    uint16_t currentDetSett = (readReg(ADS131_CFG) << 8) >>8;
    
    uint16_t newRegData = (delayRegData << 12) + (enabled << 8) + currentDetSett;

    return writeReg(ADS131_CFG, newRegData);
}

uint32_t ADS131M08::wreg(uint8_t reg, uint16_t data) {
    uint8_t cmd = 0x06;
    uint16_t cmdWord = (cmd << 12) + (reg << 7);

    //write CS LOW
    //begin transaction

    tword(cmdWord);
    tword(data);

    //send 4 empty words
    for(uint8_t i=0; i<4; i++) {
        tword();
    }

    //end transaction
    //write CS HIGH

    //get response
    uint32_t responseArr[10];
    frame(&responseArr[0]);

    if ( ( (0x04<<12) + (reg<<7) ) == responseArr[0]) {
        return true;
    } else {
        return false;
    }
}

uint32_t ADS131M08::rreg(uint8_t reg) {
    uint8_t cmd = 0x0A;
    uint16_t cmdWord = (cmd << 12) + (reg << 7);

    uint32_t responseArr[10];

    //send command
    frame(&responseArr[0], cmdWord);

    //read response
    frame(&responseArr[0]);

    return responseArr[0] >> 16;
}


uint32_t ADS131M08::tword(uint16_t bytes) {
    const struct spi_buf buf[2] = { 
        {
            .buf = bytes,
            .len = 2
        }, 
        {
            .buf = 0x00,
            .len = 1
        }
    };
    const struct spi_buf_set tx = {
        .buffers = buf
    };

    const struct spi_buf_set rx = {
        { 
            .buf = 0x000000,
            .len = 3
        }
    }

    spi_transceive_dt(&device->config, &tx, &rx);
    
}

uint16_t ADS131M08::frame(uint32_t *outPtr, uint16_t cmd) {
    //write CS LOW
    //begin transaction

    *outPtr = tword(cmd);

    // For the next 8 words, just read the data
    for(uint8_t i=1; i<9; i++) {    
        outPtr++;
        *outPtr = tword() >> 8;
    }   

    //save CRC bits
    outPtr++;
    *outPtr = tword();

    //end transaction
    //write CS high

}


bool ADS131M08::writeReg(uint8_t reg, uint16_t data) {
    /* Writes the content of data to the register reg
        Returns true if successful
    */
    
    uint8_t commandPref = 0x06;

    // Make command word using syntax found in data sheet
    uint16_t commandWord = (commandPref<<12) + (reg<<7);

    digitalWrite(CS, LOW);
    spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));

    spiTransferWord(commandWord);
    
    spiTransferWord(data);

    // Send 4 empty words
 
    for (uint8_t i=0; i<4; i++) {
        spiTransferWord();
    }

    spi->endTransaction();
    digitalWrite(CS, HIGH);

    // Get response
    uint32_t responseArr[10];
    spiCommFrame(&responseArr[0]);

    if ( ( (0x04<<12) + (reg<<7) ) == responseArr[0]) {
        return true;
    } else {
        return false;
    }
}

uint16_t ADS131M08::readReg(uint8_t reg) {
    /* Reads the content of single register found at address reg
        Returns register value
    */
    
    uint8_t commandPref = 0x0A;

    // Make command word using syntax found in data sheet
    uint16_t commandWord = (commandPref << 12) + (reg << 7);

    uint32_t responseArr[10];
    // Use first frame to send command
    spiCommFrame(&responseArr[0], commandWord);

    // Read response
    spiCommFrame(&responseArr[0]);

    return responseArr[0] >> 16;
}

uint32_t ADS131M08::spiTransferWord(uint16_t inputData) {
    /* Transfer a 24 bit word
        Data returned is MSB aligned
    */ 

    uint32_t data = spi->transfer(inputData >> 8);
    data <<= 8;
    data |= spi->transfer((inputData<<8) >> 8);
    data <<= 8;
    data |= spi->transfer(0x00);

    return data << 8;
}

void ADS131M08::spiCommFrame(uint32_t * outPtr, uint16_t command) {
    // Saves all the data of a communication frame to an array with pointer outPtr


    /*
    spi_write(device *dev, spi_config *config, spi_bf_set *tx_bufs)
    spi_read(device *dev, spi_config *config, spi_buf_set *rx_bufs)
    
    there are _async functions with struct k_poll_signal *async as the end argument
    
    spi_release(device *dev, spi_config *config)

    */

    digitalWrite(CS, LOW);

    spi->beginTransaction(SPISettings(SpiClk, MSBFIRST, SPI_MODE1));

    // Send the command in the first word
    *outPtr = spiTransferWord(command);

    // For the next 8 words, just read the data
    for (uint8_t i=1; i < 9; i++) {
        outPtr++;
        *outPtr = spiTransferWord() >> 8;
    }

    // Save CRC bits
    outPtr++;
    *outPtr = spiTransferWord();

    spi->endTransaction();

    digitalWrite(CS, HIGH);
}

int32_t ADS131M08::twoCompDeco(uint32_t data) {
    // Take the two's complement of the data

    data <<= 8;
    int32_t dataInt = (int)data;

    return dataInt/pow(2,8);
}

bool ADS131M08::setGain(int gain) { // apply gain to all channels (1 to 128, base 2 (1,2,4,8,16,32,64,128))
    uint16_t writegain = 0;
    if(gain == 1 ) {
        writegain = 0b0000000000000000;
    }  
    else if (gain == 2) {
        writegain = 0b0001000100010001;
    }
    else if (gain == 4) {
        writegain = 0b0010001000100010;
    }
    else if (gain == 8) { 
        writegain = 0b0011001100110011;
    }
    else if (gain == 16) { 
        writegain = 0b0100010001000100;
    }
    else if (gain == 32) {
        writegain = 0b0101010101010101;
    }
    else if (gain == 64) {
        writegain = 0b0110011001100110;
    }
    else if (gain == 128) {
        writegain = 0b0111011101110111;
    }
    else {
        return false;
    }
    writeReg(ADS131_GAIN1, writegain);
    writeReg(ADS131_GAIN2, writegain);

    return true;
}

