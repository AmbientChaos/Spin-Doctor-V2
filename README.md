# Spin-Doctor v2

Attempt at a translational drift platform using only telemetry from the brushless ESCs for speed/location data

**Receiver, and ESC code components all tested and functioning, melty algorithms may not be fully functional yet.**

Thanks to [@swallenhardware](https://github.com/swallenhardware) for providing his code and build log which helped make this possible

## External libraries used:

[CPPM-RX](https://github.com/daPhoosa/CPPM-RX)

## Hardware used:

Teensy 3.2 microcontroller

SparkFun H3LIS331DL accelerometer (optional)

6 channel CPPM receiver (and transmitter)

DShot600 compatible ESC with telemetry pad

750kv brushless motors

5V power converter

## Usage:
The gear and flap switches are used to determine the operating mode, with a safe mode for signal loss or startup in active modes

**Gear:** *N/A* **Flaps:** *N/A* **Mode:** safe mode        **Lights:** Fast green flashing **Notes:** Signal loss or started in mode other than idle, motor off

**Gear:** *off* **Flaps:** *off* **Mode:** drive mode       **Lights:** Solid green         **Notes:** Connected, drive mode
	
**Gear:** *off* **Flaps:** *on*  **Mode:** drive/melty mode **Lights:** Solid green and red **Notes:** Green and red lights indicate reference and movement directions respectively

**Gear:** *on*  **Flaps:** *off* **Mode:** unused	    **Lights:** NA	            **Notes:** NA

**Gear:** *on*  **Flaps:** *on*  **Mode:** max spin mode    **Lights:** Flash green and red **Notes:** Only available while spinning in melty mode

#### Safe mode:
Only entered before connecting and setting switches for idle mode and during signal loss.

Motors set off.  Safe mode after powerup.

#### Drive mode:
Motors used for tank drive.

#### Drive/Melty mode:
Throttle high on entering melty mode initiates juke maneufer first.

Rudder high on entering melty mode flips orientation for inverted driving

Throttle down for drive mode, throttle up to begin MeltyBrain drive.

Green light blinks for persistance of vision while spinning to indicate the heading of the robot.  The green lights are meant to be pointed away from the user as a point of reference, so the bot can then translate based on the receiver inputs.

Red light is used to indicate the direction given by the receiver inputs.

Rudder is used to rotate the reference light to point away from user.

Throttle controls throttle.

Aileron makes bot strafe left and right.

Elevator makes bot drive forward and backward.

Accelerometer used to account for flipping.

#### Max spin mode:

Melty mode with the speed set to maximum instead of a lower speed for better translation.

Intended to be activated in the middle of an arena to give maximum energy storage at the expense of mobility.
