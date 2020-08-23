#include "Accelerometer.h"
#include "DShot.h"
#include "Melty.h"
#include "Watchdog.h"

//define pins
#define GREEN 23
#define RED 22
#define ACCEL_SCL 19
#define ACCEL_SDA 18
#define RX_CPPM 16
#define ESC_1 9 //FTM0_CH2
#define ESC_2 10 //FTM0_CH3

//define states
#define STATE_IDLE 0
#define STATE_DRIVE 1
#define STATE_JUKE 2
#define STATE_MELTY 3
#define STATE_MAX_SPIN 4
#define STATE_CALIBRATE 5
byte state = STATE_IDLE;
byte prevState = STATE_IDLE;
const bool useAccel = false;

unsigned long blinkTimer = millis();
bool blinkBlock = false;

Receiver r;

void setup() {
  StartCPPM(RX_CPPM);
  if (useAccel) accelSetup();
  Serial1.begin(115200); // open Serial1 for ESC telemetry
  readCalibration();
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);

  //setup ESC DShot out
  pinMode(ESC_1, OUTPUT);
  digitalWrite(ESC_1, LOW);
  pinMode(ESC_2, OUTPUT);
  digitalWrite(ESC_2, LOW);
  setupDshotDMA();
  dshotOut(0, 1); //arm motor
  dshotOut(0, 2); //arm motor

  //setup watchdog
  watchdogSetup();

  state = STATE_IDLE;
}

void loop() {
  feedWatchdog();

  //check for new receiver inputs
  if (Aux1New()){
    readReceiver(r);
    //check for state changes
    stateChange();
  }

  //get any new accelerometer data
  if (useAccel && accelNew) runAccel();

  switch (state) {
    case STATE_IDLE:
      //fast blink green status light to warn of improper startup switches or lost signal
      if (millis() - blinkTimer > 200) {
        blinkTimer = millis();
        digitalWrite(GREEN, !digitalRead(GREEN));
        digitalWrite(RED, LOW);
      }
      //send motor stop command
      setMotor(0, 1);
      setMotor(0, 2);
      break;

    case STATE_DRIVE:
      //green LED solid to show drive mode
      //red LED solid with throttle high to show juke maneuver on entering melty mode (if 2 wheels)
      drive(r);
      break;

    case STATE_JUKE:
      // red light while executing juke maneuver
      jukeFinished = false;
      jukeStartTime = millis();
      juke(r);
      if (jukeFinished) state = STATE_MELTY;
      break;
    
    case STATE_MELTY:
      //both red and green LED solid while not spinning to show melty mode
      maxSpin = false;
      calibrating = false;
      runMelty(r);
      break;

    case STATE_MAX_SPIN:
      //LEDs blinking to show max spin mode
      maxSpin = true;
      calibrating = false;
      runMelty(r);
      if (millis() - blinkTimer > 150) { // cycle melty LEDs at ~3Hz
        blinkTimer = millis();
        blinkBlock = !blinkBlock;
      }
      if (blinkBlock) {
        digitalWrite(GREEN, LOW);
        digitalWrite(RED, LOW);        
      }
      break;

    case STATE_CALIBRATE:
      //red LED solid heading to show calibration mode
      maxSpin = false;
      calibrating = true;
      r.ailer = 500;
      r.elev = 500;
      runMelty(r);
      digitalWrite(RED, digitalRead(GREEN));
      digitalWrite(GREEN, LOW);
      break;

    default:
      break;
  }
}

void stateChange() {
  //check switches to determine program state to transition to
  prevState = state;
  //safe state if transmitter connection is lost
  if (signalLost()) state = STATE_IDLE;
  //drive if both switches off
  else if (r.flap <= 500 && r.gear <= 500) state = STATE_DRIVE;
  //drive if flap switch is on but throttle is down
  else if (r.flap > 500 && r.gear <= 500 && r.throt < 150) state = STATE_DRIVE;
  //melty if flap switch is on and throttle is up
  else if (r.flap > 500 && r.gear <= 500 && r.throt >= 150) state = STATE_MELTY;
  //max spin if both switches on
  else if (r.flap > 500 && r.gear > 500) state = STATE_MAX_SPIN;
  //calibrate mode if gear switch is on
  else if (r.flap <= 500 && r.gear > 500) state = STATE_CALIBRATE;

  //check state transitions
  //idle can only exit to drive when both switches off and throttle is down
  if (prevState == STATE_IDLE && state == STATE_DRIVE && r.throt > 150) state = STATE_IDLE;
  //idle can only exit to drive mode
  else if (prevState == STATE_IDLE && state != STATE_DRIVE) state = STATE_IDLE;
  //entering melty mode with throttle high initiates a juke maneuver
  else if (prevState == STATE_DRIVE && state == STATE_MELTY && r.throt > 500) state = STATE_JUKE;
  //entering melty mode with rudder high flips the controks for inverted driving
  else if (prevState == STATE_DRIVE && state == STATE_MELTY && r.rudd > 750) flipped = !flipped;
  //Max spin mode cannot be entered at low throttle to prevent accidents
  else if (prevState == STATE_MELTY && state == STATE_MAX_SPIN && r.throt < 250) state = STATE_MELTY;
  //max spin mode can only exit to melty mode or drive (or idle)
  else if (prevState == STATE_MAX_SPIN && state != STATE_MAX_SPIN) {
    if (state == STATE_MELTY || state == STATE_DRIVE || state == STATE_IDLE);
    else state = STATE_MAX_SPIN;
  }
  //calibrate mode can only exit to drive mode (or idle), saving calibration to EEPROM
  else if (prevState == STATE_CALIBRATE && state != STATE_CALIBRATE) {
    if (state == STATE_DRIVE || state == STATE_IDLE) writeCalibration();
    else state = STATE_MAX_SPIN;
  }
}
