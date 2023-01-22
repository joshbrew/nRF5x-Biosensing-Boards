#include "ADS131M08.h"
#include "esp_timer.h"

//Joshua Brewster, Jacob Tinkhauser

#define MISO_PIN              19
#define MOSI_PIN              23
#define SCK_PIN               18
#define CS_PIN                5

#define VSPI_MISO             MISO_PIN
#define VSPI_MOSI             MOSI_PIN
#define VSPI_SCLK             SCK_PIN
#define VSPI_SS               CS_PIN

#define DRDY_PIN              21
#define XTAL_PIN              22
#define RESET                 17

int CLKOUT  =                 8192000; //XTAL speed (50% duty cycle PWM)

ADS131M08 adc(CS_PIN,XTAL_PIN,DRDY_PIN,8192000);

unsigned long long sampleNum = 0;
unsigned long long currentMicros = 0;
char outputarr[128];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial.println("Setting SPI");
  adc.spi = new SPIClass(VSPI);
  Serial.println("Booting SPI");
  adc.init(CLKOUT);

  delay(100);


  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(1);
  digitalWrite(RESET, HIGH);

  adc.writeReg(ADS131_CLOCK,0b1111111100011010); //Clock register (page 55 in datasheet)
  /*CLOCK REG SETTINGS
   * Bytes 15-8: ADC Channel enable/disable
   * Byte 7: Crystal disable 
   * Byte 6: External Reference Enable
   * Byte 5: Reserved
   * Bytes 4-2: Modulator Oversampling 000 = 128 OSR (32ksps), 111 = 16256 OSR (250sps)
   * Bytes 1-0: Power mode selections 11 or 10 = high resolution, 01 = low power, 00 = very low power
   */

  //adc.writeReg(ADS131_CFG,0b0000000000000000);

  //DC Block Filter settings:
  //adc.writeReg(ADS131_THRSHLD_LSB,0b0000000000001010);

  //Channel settings
  //adc.writeReg(ADS131_CH0_CFG,0b0000000000000000);

  //Gain settings, 1-128 (increasing by factor of 2)
  adc.setGain(32);

   
  uint16_t clkreg = adc.readReg(ADS131_CLOCK);
  Serial.print("CLOCK: ");
  Serial.println(clkreg,BIN);
  

  
  uint16_t gainreg = adc.readReg(ADS131_GAIN1);
  Serial.print("GAIN1: ");
  Serial.println(gainreg, BIN);
  
  //adc.globalChop(true,2);
  
  uint16_t id = adc.readReg(ADS131_ID);
  Serial.print("ID: ");
  Serial.println(id, BIN);
  
  uint16_t stat = adc.readReg(ADS131_STATUS);
  Serial.print("Status: ");
  Serial.println(stat, BIN);

  uint16_t Mode = adc.readReg(ADS131_MODE);
  Serial.print("Mode: ");
  Serial.println(Mode, BIN);

  Serial.println("Starting in 3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);


  currentMicros = esp_timer_get_time();

}


void loop() {
 if(digitalRead(DRDY_PIN)) {

    //Serial.println(adc.readChannelSingle(0)); //Just read the first one
    int32_t channelArr[8];
    adc.readAllChannels(channelArr);
    currentMicros = esp_timer_get_time();
    sprintf(outputarr, "%d,%d,%d,%d,%d,%d,%d,%d,%u\r\n", 
              channelArr[0], 
              channelArr[1], 
              channelArr[2],
              channelArr[3],
              channelArr[4],
              channelArr[5],
              channelArr[6],
              channelArr[7],
              currentMicros
              );
              
    Serial.print(outputarr);
//    Serial.print(channelArr[0]);
//    Serial.print("|");
//    Serial.print(currentMicros);
//    Serial.print("\r\n");
    sampleNum++;
  }
  else {
  }
}
