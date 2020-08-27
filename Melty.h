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

//Angle calculation algorithm derived from Halo codebase (https://github.com/swallenhardware/MeltyHalo)

#include "Receiver.h"
#include "Telemetry.h"

//melty variables
uint16_t throtCurrent = 0;
uint16_t movementDirection = 0;
uint16_t movementSpeed = 0;
bool maxSpin = false;
bool calibrating = false;
uint16_t telemAngle[2] = {0}; //current angle calculated from accelerometer
uint16_t currentAngle = 0;
int8_t flipped = 1;

//manually calibrated variables
const uint16_t throtMin = 100; // minimum throttle to start spinning, out of 1000
const uint16_t throtMax = 300; // maximum throttle for melty mode, out of 1000
const uint16_t lightOffset = 45; //angle between lights and "front", which is 90 deg offset from the motor axle

//timers
unsigned long lastMotor1Send = micros();
unsigned long lastMotor2Send = micros();
unsigned long telemDelay = micros();
unsigned long trimTimer = millis();

//define pins
#define GREEN 4
#define RED 6

//juke variables
bool jukeFinished = true;
const uint16_t jukeTurnTime = 100; // turn time in msec
const uint16_t jukeDriveTime = 500; // drive time in msec
unsigned long jukeStartTime = millis();

void trimAngle() {
  //use rudder to rotate heading direction
  static int16_t angleTrim;
  if (millis() - trimTimer > 25) {
    trimTimer = millis();
    angleTrim = (rudd - 500) / 100;
  }
  for (static int i = 0; i < 2; i++) {
    telemAngle[i] = (telemAngle[i] + 360 + angleTrim) % 360;
  }
}

void calibrateSpeed(){
  if (millis() - trimTimer > 330) {
    trimTimer = millis();
    if (rudd >= 750) periodOffset--;
    else if (rudd <= 250) periodOffset++;
  }
}

void getRadial() {
  //calculate melty inputs from receiver
  if (newInput) { 
    newInput = false;
    movementSpeed = min(500, (int)hypot(ailer - 500, elev - 500));
    movementDirection = (atan2((ailer - 500) * flipped, elev - 500) * 4068) / 71; //deg = rad * 4068 / 71
    throtCurrent = maxSpin ? 1000 : (uint32_t) throt * throtMax / 1000;
    if (calibrating) calibrateSpeed();
    else trimAngle();
  }
}

void getAngle() { 
  //triangular integration calculations borrowed from Halo
  static uint16_t deltaT;
  //calculate angle from new telem data
  if (telemNew) { //if new telemetry, triangular integration from new data
    telemNew = false;
    telemAngle[1] = telemAngle[0]; //shift old data down
    deltaT = telemTime[0] - telemTime[1];
    telemAngle[0] = (telemAngle[1] + (deltaT / degreePeriod[0] + deltaT / degreePeriod[1]) / 2) % 360;
    currentAngle = telemAngle[0];
  }
  else { //if no new telemetry, predict the angle between telem readings by extrapolating from old data
    static uint16_t newTime = micros();
    static uint16_t periodPredicted = degreePeriod[1] + (newTime - telemTime[1]) * (degreePeriod[0] - degreePeriod[1]) / (telemTime[0] - telemTime[1]);
    //predict the current robot heading by triangular integration up to the extrapolated point
    deltaT = newTime - telemTime[0];
    currentAngle = (telemAngle[0] + (deltaT / periodPredicted + deltaT / degreePeriod[0]) / 2) % 360;
  }
}

void meltLights() {
  //turn on green light if it's position is "forward"
  if ((currentAngle + 360 + lightOffset) % 360 <= 30 
      || (currentAngle + 360 + lightOffset) % 360 >= 330) {
    digitalWriteFast(GREEN, HIGH);
  }
  else {
    digitalWriteFast(GREEN, LOW);
  }
  //turn on red ligth if its position is in the stick direction
  if (movementSpeed > 50) {
    if ((currentAngle + 360 + lightOffset) % 360 <= (movementDirection + 30) % 360 
        || (currentAngle + 360 + lightOffset) % 360 >= (movementDirection - 30) % 360) {
      digitalWriteFast(RED, HIGH);
    }
    else {
      digitalWriteFast(RED, LOW);
    }
  }
}

void setMotor(int16_t value, uint8_t motor = 1) {
  //send DShot command based on -1000 to 1000 throttle input
  //limit max update rate to 4kHz, min 25us between motors
  static bool telem = false;
  if (((motor == 1) && (micros() - lastMotor1Send >= 250) &&(micros() - lastMotor2Send >= 30)) 
    ||((motor == 2) && (micros() - lastMotor2Send >= 250) &&(micros() - lastMotor1Send >= 30))) { 
    if (motor == 1 && micros() - telemDelay > 900) {
      telemDelay = micros();
      telem = true;
    }
    if(value == 0) {}
    else if(value > 0) {value = value * 1 + 1047;}
    else if(value < 0) {value = value * -1 + 47;}
    else {value = 0;}
    dshotOut(value, motor, telem);
    telem = false;
    if(motor == 1) {lastMotor1Send = micros();}
    else if(motor == 2) {lastMotor2Send = micros();}
  }
  else {return;}
}

void drive() {
  digitalWriteFast(GREEN, HIGH);
  if (throt > 500) digitalWriteFast(RED, HIGH);
  else digitalWriteFast(RED, LOW);
  if (elev < 520 && elev > 480) elev = 500;
  if (ailer < 520 && ailer > 480) ailer = 500;
  setMotor(flipped * (elev + ailer - 1000), 1);
  setMotor(flipped * (elev - ailer), 2);
}

void juke(){
  static unsigned long jukeTime = millis() - jukeStartTime;
  if (jukeTime <= jukeTurnTime) ailer = 1000;
  else if (jukeTime <= jukeDriveTime) {
    ailer = 500;
    elev = 1000;
  }
  else {
    ailer = 500;
    elev = 500;
    jukeFinished = true;
  }
  drive();
}

void meltMove() {
  static int16_t diff;
  //translate if stick is moved enough
  if (movementSpeed > 50) {
    diff = 180 - abs(abs(movementDirection - currentAngle) - 180);
    //speed up motor if moving toward movement direction
    if (diff < 90) {
      setMotor(flipped * (throtCurrent), 1);
      setMotor(flipped * (throtCurrent * 8 / 10), 2);
    }
    //slow down motor if moving away from movement direction
    else {
      setMotor(flipped * (throtCurrent * 8 / 10), 1);
      setMotor(flipped * (throtCurrent), 2);
    }
  }
  //spin in place if no stick movement
  else {
    setMotor(flipped * throtCurrent, 1);
    setMotor(flipped * throtCurrent, 2);
  }
}

void runMelty() {
  getRadial();

  //run melty if throttle is high enough
  if (throt >= throtMin) {
    getAngle();
    meltLights();
    meltMove();
  }
  //throttle too low for translation, shut off motor
  else { 
    drive();
  }
}
