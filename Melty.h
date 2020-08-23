#include "Receiver.h"
#include "Telemetry.h"

//melty variables
uint16_t throtCurrent = 0;
uint16_t movementDirection = 0;
uint16_t movementSpeed = 0;
bool maxSpin = false;
bool calibrating = false;
uint16_t telemAngle[arraySize] = {0}; //current angle calculated from accelerometer

//manually calibrated variables
const uint16_t throtMin = 100; // minimum throttle to start spinning, out of 1000
const uint16_t throtMax = 400; // maximum throttle for melty mode, out of 1000
const uint16_t lightOffset = 45; //angle between lights and "front", which is 90 deg offset from the motor axle

uint16_t currentAngle = 0;
unsigned long lastMotor1Send = micros();
unsigned long lastMotor2Send = micros();
unsigned long trimTimer = millis();

//define pins
#define GREEN 23
#define RED 22

//juke variables
bool jukeFinished = true;
const uint16_t jukeTurnTime = 100; // turn time in msec
const uint16_t jukeDriveTime = 500; // drive time in msec
unsigned long jukeStartTime = millis();

void trimAngle(Receiver r) {
  //use rudder to rotate heading direction
  static int16_t angleTrim;
  if (millis() - trimTimer > 25) {
    trimTimer = millis();
    angleTrim = (r.rudd - 500) / 100;
  }
  for (static int i = 0; i < arraySize; i++) {
    telemAngle[i] = (telemAngle[i] + angleTrim) % 360;
  }
}

void calibrateSpeed(Receiver r){
  if (millis() - trimTimer > 250) {
    trimTimer = millis();
    if (r.rudd >= 750) periodOffset--;
    else if (r.rudd <= 250) periodOffset++;
  }
}

void getRadial(Receiver r) {
  //calculate melty inputs from receiver
  if (r.newInput) { 
    r.newInput = false;
    movementSpeed = min(500, (int)hypot(r.ailer - 500, r.elev - 500));
    movementDirection = (atan2((r.ailer - 500) * flipped, r.elev - 500) * 4068) / 71; //deg = rad * 4068 / 71
    throtCurrent = maxSpin ? 1000 : (uint32_t) r.throt * throtMax / 1000;
    if (calibrating) calibrateSpeed(r);
    else trimAngle(r);
  }
}

void getAngle() { 
  //triangular integration calculations borrowed from Halo
  static uint16_t deltaT;
  //calculate angle from new telem data
  if (telemNew) { 
    telemNew = false;
    
    //shift old data down
    for (static int i = 1; i < arraySize; i++) { 
      telemAngle[i] = telemAngle[i - 1];
    }
    
    //triangular integration from new data
    deltaT = telemTime[0] - telemTime[1];
    telemAngle[0] = (telemAngle[1] + (deltaT / degreePeriod[0] + deltaT / degreePeriod[1]) / 2) % 360;
    currentAngle = telemAngle[0];
  }
  //predict the angle between telem readings by extrapolating from old data
  else { 
    static uint16_t newTime = micros();
    static uint16_t periodPredicted = degreePeriod[1] + (newTime - telemTime[1]) * (degreePeriod[0] - degreePeriod[1]) / (telemTime[0] - telemTime[1]);
    //predict the current robot heading by triangular integration up to the extrapolated point
    deltaT = newTime - telemTime[0];
    currentAngle = (telemAngle[0] + (deltaT / periodPredicted + deltaT / degreePeriod[0]) / 2) % 360;
  }
}

void meltLights() {
  //turn on green light if it's position is "forward"
  if ((currentAngle + lightOffset) % 360 <= 10 
      || (currentAngle + lightOffset) % 360 >= 350) {
    digitalWrite(GREEN, HIGH);
  }
  else {
    digitalWrite(GREEN, LOW);
  }
  //turn on red ligth if its position is in the stick direction
  if (movementSpeed > 50) {
    if ((currentAngle + lightOffset) % 360 <= (movementDirection + 10) % 360 
        || (currentAngle + lightOffset) % 360 >= (movementDirection - 10) % 360) {
      digitalWrite(RED, HIGH);
    }
    else {
      digitalWrite(RED, LOW);
    }
  }
}

void setMotor(int16_t value, uint8_t motor = 1) {
  //send DShot command based on -1000 to 1000 throttle input
  //ESC is expected to be flashed to be in Bidirectional 3D mode
  //limit max update rate to 4kHz, min 25us between motors
  if (((motor == 1) && (micros() - lastMotor1Send >= 250) &&(micros() - lastMotor2Send >= 25)) 
    ||((motor == 2) && (micros() - lastMotor2Send >= 250) &&(micros() - lastMotor1Send >= 25))) { 
    if(value == 0) dshotOut(value, motor);
    else {
      if(value > 0) dshotOut(value * 1 + 1047, motor);
      else if(value < 0) dshotOut(value * -1 + 47, motor);
    }
    if(motor == 1) lastMotor1Send = micros();
    else if(motor == 2) lastMotor2Send = micros();
  }
}

void drive(Receiver r) {
  digitalWrite(GREEN, HIGH);
  if (r.throt > 500) digitalWrite(RED, HIGH);
  else digitalWrite(RED, LOW);
  setMotor(flipped * (r.elev + r.ailer - 1000), 1);
  setMotor(flipped * (r.elev - r.ailer), 2);
}

void juke(Receiver r){
  static unsigned long jukeTime = millis() - jukeStartTime;
  if (jukeTime <= jukeTurnTime) r.ailer = 1000;
  else if (jukeTime <= jukeDriveTime) {
    r.ailer = 500;
    r.elev = 1000;
  }
  else {
    r.ailer = 500;
    r.elev = 500;
    jukeFinished = true;
  }
  drive(r);
}

void meltMove() {
  static int16_t diff;
  //translate if stick is moved enough
  if (movementSpeed > 50) {
    diff = 180 - abs(abs(movementDirection - currentAngle) - 180);
    //speed up motor if moving toward movement direction
    if (diff < 90) {
      setMotor(flipped * (throtCurrent), 1);
      setMotor(flipped * (max(throtCurrent - 200,0)), 2);
    }
    //slow down motor if moving away from movement direction
    else {
      setMotor(flipped * (max(throtCurrent - 200,0)), 1);
      setMotor(flipped * (throtCurrent), 2);
    }
  }
  //spin in place if no stick movement
  else {
    setMotor(flipped * throtCurrent, 1);
    setMotor(flipped * throtCurrent, 2);
  }
}

void runMelty(Receiver r) {
  getRadial(r);
  //receiveTelemetry();

  //run melty if throttle is high enough
  if (throtCurrent >= throtMin) {
    getAngle();
    meltLights();
    meltMove();
  }
  //throttle too low for translation, shut off motor
  else { 
    drive(r);
  }
}
