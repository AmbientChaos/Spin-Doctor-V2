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
#define STATE_STARTUP 0
#define STATE_DRIVE 1
#define STATE_JUKE 2
#define STATE_MELTY 3
#define STATE_MAX_SPIN 4
byte state = STATE_STARTUP;
byte prevState = STATE_STARTUP;
bool useAccel = false;

unsigned long blinkTimer = millis();
bool blinkBlock = false;

Receiver r;

void setup() {
  //setup receiver interrupt
  StartCPPM(RX_CPPM);

  //setup accelerometer
  if (useAccel) accelSetup();

  //open receiver telemetry port
  Serial1.begin(115200); // open Serial1 for ESC communication

  //setup LEDs
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
    case STATE_STARTUP:
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
      state = STATE_MELTY;
      break;
    
    case STATE_MELTY:
      //both red and green LED solid while not spinning to show melty mode
      maxSpin = false;
      runMelty(r);
      break;

    case STATE_MAX_SPIN:
      //LEDs blinking to show max spin mode
      maxSpin = true;
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

    default:
      break;
  }
}

void stateChange() {
  //check switches to determine program state to transition to
  prevState = state;
  //safe state if transmitter connection is lost
  if (signalLost()) state = STATE_STARTUP;
  //drive if both switches off
  else if (r.flap <= 500 && r.gear <= 500) state = STATE_DRIVE;
  //melty if flap switch is on and throttle is up
  else if (r.flap > 500 && r.gear <= 500 && r.throt >= 100) state = STATE_MELTY;
  //max spin if both switches on
  else if (r.flap > 500 && r.gear > 500) state = STATE_MAX_SPIN;

  //check state transitions
  //startup can only exit to idle when both switches off and throttle is down
  if (prevState == STATE_STARTUP && state == STATE_DRIVE && r.throt > 100) state = STATE_STARTUP;
  //startup can only exit to drive mode
  else if (prevState == STATE_STARTUP && state != STATE_DRIVE) state = STATE_STARTUP;
  //entering melty mode with throttle high initiates a juke maneuver
  else if (prevState == STATE_DRIVE && state == STATE_MELTY && r.throt > 500) state = STATE_JUKE;
  //entering melty mode with throttle high initiates a juke maneuver
  else if (prevState == STATE_DRIVE && state == STATE_MELTY && r.rudd > 750) flipped = !flipped;
  //Max spin mode cannot be entered at low throttle to prevent accidents
  else if (prevState == STATE_MELTY && state == STATE_MAX_SPIN && r.throt < 250) state = STATE_MELTY;
  //max spin mode can only exit to melty mode or drive
  else if (prevState == STATE_MAX_SPIN && state != STATE_MAX_SPIN) {
    if (state == STATE_MELTY || state == STATE_DRIVE);
    else state = STATE_MAX_SPIN;
  }
}
