/*SpinDoctor - Software for sensorless translational drift
Copyright (C) 2020  AmbientChaos

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.*/

#include <EEPROM.h>

//telemetry variables
bool telemNew = true;
bool telemProcessed = false;
unsigned long telemNewTime = 0;
unsigned long telemTime[2] = {0}; //time between accelerometer measurements
uint16_t degreePeriod[2] = {0}; //period in microseconds per degree
uint8_t SerialBuf[10];
uint8_t receivedBytes = 0;
int8_t mem = 42;
int16_t periodOffset = 0;

// Manually calibrated variables 
const uint8_t wheelDiam = 41; // wheel diam in mm
const uint8_t circleDiam = 60; // wheel circle diam in mm
const uint8_t motorPoles = 14;

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
  if (EEPROM.read(0) == mem) periodOffset = EEPROM.read(1) | EEPROM.read(2) << 8;
  else periodOffset = 0;
}

void writeCalibration(){
  //write validation byte and calibration data to EEPROM
  EEPROM.update(0, mem); //fill bit to flag as good calibration data
  EEPROM.update(1, lowByte(periodOffset));
  EEPROM.update(2, highByte(periodOffset));
}

void serialEvent1(){
  while(Serial1.available()){
    if(receivedBytes <=9){
      SerialBuf[receivedBytes] = Serial1.read();
      receivedBytes++;
    }
    if(receivedBytes > 9){ // transmission complete
      telemNew = true;
      telemProcessed = false;
      telemNewTime = micros();
      break;
    }
  }
}

void processTelemetry(){
  static uint16_t eRpm = 0;
  static uint16_t motorRpm = 0;
  static uint16_t ringRpm = 0;
  telemProcessed = true;
  receivedBytes = 0;
  
  // CRC8 check failing most of the time with CRC8 of 0, disabled
  //static uint8_t crc = get_crc8(SerialBuf, 9); // get the 8 bit CRC
  //if(crc != SerialBuf[9]) return; // transmission failure
  
  //shift old data down
  telemTime[1] = telemTime[0];
  degreePeriod[1] = degreePeriod[0];
  
  //record new data
  telemTime[0] = telemNewTime;
     
  // compute the received values
  /*ESC_telemetry[0] = SerialBuf[0]; // temperature
  ESC_telemetry[1] = (SerialBuf[1]<<8)|SerialBuf[2]; // voltage
  ESC_telemetry[2] = (SerialBuf[3]<<8)|SerialBuf[4]; // Current
  ESC_telemetry[3] = (SerialBuf[5]<<8)|SerialBuf[6]; // used mA/h
  ESC_telemetry[4] = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100*/
  eRpm = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100
  
  //calculate rotation speed(period) from telemetry
  motorRpm = (uint32_t) eRpm * 100 / (motorPoles / 2);
  ringRpm = (uint32_t) motorRpm * wheelDiam / circleDiam;
  degreePeriod[0] = (55556 / ringRpm) * 3 + periodOffset; // convert from rev/min to usec/deg (in 16 bit integer math)
}
