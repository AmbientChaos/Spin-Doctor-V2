//accelerometer variables
bool telemNew = true;
unsigned long telemNewTime = 0;
const byte arraySize = 2; //2 for triangular integration
unsigned long telemTime[arraySize] = {0}; //time between accelerometer measurements
uint16_t degreePeriod[arraySize] = {0}; //period in microseconds per degree
uint16_t telemAngle[arraySize] = {0}; //current angle calculated from accelerometer

static uint8_t receivedBytes = 0;
uint16_t eRpm = 0;
uint16_t motorRpm = 0;
uint16_t ringRpm = 0;
uint8_t wheelDiam = 25; // wheel diam in mm
uint8_t circleDiam = 100; // wheel circle diam in mm

// 8-Bit CRC
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}


// get the Telemetry from the ESC
void receiveTelemetry(){
  static uint8_t SerialBuf[10];
  
  while(Serial1.available() && receivedBytes <= 9){ // collect bytes
    SerialBuf[receivedBytes] = Serial1.read();
    receivedBytes++;
  }
  if(receivedBytes == 10){ // transmission complete
    uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC
    if(crc8 != SerialBuf[9]){  // transmission failure
      for (int i = 0; i < 9; i++){SerialBuf[i] = SerialBuf[i+1];}
      receivedBytes--;
      return;
    }
  
    //shift old data down
    for(int i=arraySize - 1; i>0; i--) { //shift old data down
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
    degreePeriod[0] = (55556 / ringRpm) * 3; // convert from rev/min to usec/deg (in 16 bit integer math)
    
    receivedBytes = 0;
    telemNew = true;
    telemNewTime = micros();
  }
}
