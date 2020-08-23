//EEPROM library for saving calibration variables
#include <EEPROM.h>

//telemetry variables
bool telemNew = true;
unsigned long telemNewTime = 0;
const byte arraySize = 2; //2 for triangular integration
unsigned long telemTime[arraySize] = {0}; //time between accelerometer measurements
uint16_t degreePeriod[arraySize] = {0}; //period in microseconds per degree
int16_t periodOffset = 0;

// Manually calibrated variables 
const uint8_t wheelDiam = 35; // wheel diam in mm
const uint8_t circleDiam = 60; // wheel circle diam in mm

// 8-Bit CRC
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  static uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  static uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}

void readCalibration(){
  //check that data is valid and read in saved calibration data from EEPROM
  if (EEPROM.read(0) == 42) periodOffset = EEPROM.read(1) | EEPROM.read(2) << 8;
  else periodOffset = 0;
}

void writeCalibration(){
  //write validation byte and calibration data to EEPROM
  EEPROM.update(0, 42); //fill bit to flag as good calibration data
  EEPROM.update(1, lowByte(periodOffset));
  EEPROM.update(2, highByte(periodOffset));
}

// get the Telemetry from the ESC
void serialEvent1(){ //void receiveTelemetry() {
  static uint8_t SerialBuf[10];
  static uint8_t receivedBytes = 0;
  static uint16_t eRpm = 0;
  static uint16_t motorRpm = 0;
  static uint16_t ringRpm = 0;
  
  while(Serial1.available()){
    if (receivedBytes <= 9){ // collect bytes
      SerialBuf[receivedBytes] = Serial1.read();
      receivedBytes++;
    }
    if(receivedBytes == 10){ // transmission complete
      static uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC
      if(crc8 != SerialBuf[9]){  // transmission failure
        for (static int i = 0; i < 9; i++){SerialBuf[i] = SerialBuf[i+1];}
        receivedBytes--;
        return;
      }
    
      //shift old data down
      for(static int i=arraySize - 1; i>0; i--) { //shift old data down
          telemTime[i] = telemTime[i-1];
          degreePeriod[i] = degreePeriod[i-1];
      }
      telemTime[0] = telemNewTime;
      
      // compute the received values
      /*ESC_telemetry[0] = SerialBuf[0]; // temperature
      ESC_telemetry[1] = (SerialBuf[1]<<8)|SerialBuf[2]; // voltage
      ESC_telemetry[2] = (SerialBuf[3]<<8)|SerialBuf[4]; // Current
      ESC_telemetry[3] = (SerialBuf[5]<<8)|SerialBuf[6]; // used mA/h
      ESC_telemetry[4] = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100*/
      eRpm = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100
      
      //calculate rotation speed(period) from telemetry
      motorRpm = eRpm * 100 / 7; // 14 poles / 2 = 7
      ringRpm = motorRpm * wheelDiam / circleDiam;
      degreePeriod[0] = (55556 / ringRpm) * 3 + periodOffset; // convert from rev/min to usec/deg (in 16 bit integer math)
      
      receivedBytes = 0;
      telemNew = true;
      telemNewTime = micros();
    }
  }
}
