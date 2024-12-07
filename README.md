# ft-robot-control
Framework to control Fischertechnik's 6-Axis-Robot

## DESCRIPTION

With its top-line computer controller TXT 4.0, ${\textsf{\color{red}fischer\color{blue}technik}}$® recently added an updated signature model to their long history of *Training Robot* kits. The new version is a 6-axis articulated arm, driven by DC motors, RC servos and pneumatics. This repository contains a Class with a number of helper methods to control fischertechnik encoder motors connected to an [ftDuino](https://github.com/harbaum/ftduino/).

### Configuration

This library was primarily written for the aforementioned 6-Axis robot and the motors and servos by fischertechnik it is equipped with. For now, most configuration is done via hard-coded values in the Motor.h header file, so in case you need to adjust anything, look into this file.

### Methods

#### void init(int motorId, int counterId, int switchId, float pulsesPerDegreeRotation, bool reversed);
Call the init function to adjust the motor methods to individual axis setup. 

  * `motorId` expects an ftDuino identifier, like `Ftduino::M1`.
  * `counterId` expects the corresponding counter input from the rotary encoder, like `Ftduino::C1`.
  * `switchId` expects a corresponding limit switch input, usually something like `Ftduino::I1`.
  * `pulsesPerDegreeRotation` depends on the used motor, motor gearbox and ft gears used to drive the joint. See the supplied control .ino for sane values.
  * `reversed` is a bool to tell us if this motor needs to be reversed. You could just as well just switch the connectors on the motor.


#### int getId()

So far, we have only implemented some getters and setters. Here is one of them: It returns the set motor `motorId`.

#### void setMinSpeed(int minSpeed)

Sets the minimum speed of our motor, `_minSpeed`, values 0-64;

#### int getMinSpeed()

Returns `_minSpeed`

#### void setMotionProfile(int motionProfile)

The library is able to drive our motor according to several "[Motion Profile](https://www.micropolis.com/support/kb/micropolis-robotics-primer#Motion-Profile
)". Check the if/else switch in `update()` to see which integer maps to which motion profile. Currently, we support raw on/off, a simple ramping algorithm, [PID controller](https://www.micropolis.com/support/kb/micropolis-robotics-primer#PID-Control)-style acceleration/deceleration, and a mode combining ramped with speed-controlled cruise phases.

#### int getMotionProfile()

Returns the set motion profile. `_motionProfile`

#### bool isCalibrated() {

Returns whether our motor has completed calibration or not. Returns `_calibrated`.

#### bool isRunning()

The internal variable `_running` is populated with a `millis()` timestamp when our motor is activated. It is used as a flag in the `update()` method. This getter returns this variable.

#### void set(int direction, int speed)

For raw drive control of the set motor, we use a wrapper around `ftduino.motor_set()`. This wrapper passes the set `motorId` along with the provided `direction` and `speed`. Additionally, it sets some of our internal variables. Therefore, use this wrapper instead of `ftduino.motor_set()` to directly drive a motor.

#### void set(int direction, int speed, int togo)

Overloaded version of set(), expecting an additional variable `togo` for the number of pulses the motor should turn.

#### void setDeg(int absDegrees, int speed)

Higher-level way to drive an axis motor. It expects `absDegrees` and `speed`, and will drive a motor from the current position to the provided angular degrees at the provided speed. For this to work, the axis has to be "calibrated" by executing the "homing motion", which is implemented in `moveHome()`.

Joints on the fischertechnik robot do not use absolute positioning but "incremental positioning" (or "closed-loop control with limit switches"). To determine the axis's angular position, we use a limit switch at one end of the rotation to establish a known reference position. From there, we can determine positions by counting pulses from a rotary encoder and knowing how far the joint travels per pulse.

#### int speedFromPosPID(int targetPosition, int currentPosition)

Returns a speed integer ranging from `_minSpeed` to `_maxSpeed` that is based on the relative distance between current position and target position as calculated by a simple PID algorithm.

A [PID controller](https://www.micropolis.com/support/kb/micropolis-robotics-primer#PID-Control) continuously adjusts motor speed based on the error between the desired and current positions. When well-tuned, it provides more accurate positioning and smoother motion compared to a ramping approach, as it better accounts for the distance that needs to be traveled.

#### int speedFromVelocityProp(int speed, float currentVelocity, float desiredVelocity)

While a PID controller integrates past and expected error, this simpler algorithm simply maps an error proportionally to our sane range of speed integers between min and max speed.

#### float getVelocity(unsigned long currentTime, int currentPosition)

Returns the current calculated velocity of the activated axis (motor) in pulses per second. This can be used to derrive the actual turn rate in angular degrees per second (deg/s) based on `_pulsesPerDegreeRotation`.

#### int speedFromDeltaRamped(int pulsesToGo, int pulsesCounted)

Returns a speed value based on the delta between how many pulses we need to go and how many we have already travelled (pulses counted). This provides a basic "trapezoidal motion profile, divided into three phases: acceleration, cruise and deceleration phase.

#### int rev(int direction)

Internal method to make the reversal of the desired turn direction more readable.

#### void update()

This method needs to be called by you from the Arduino `loop()` as often as possible. It checks if the limit switch is engaged or if an active motor is over a supplied time budget - both of these situations are regarded as errors and will issue a motor brake. If both of these are false, `update()` checks if the motor needs to be throttled or brought to a halt in case we reach the desired encoder position.

Here we have the core of this library, as we have decoupled activating and running a motor from waiting for the motor to reach a set position. This way, we are able to drive multiple motors at once and monitor their status in "parallel". Otherwise, we would be required to drive one axis of our robot after another.

#### void report()

Simple debug method that prints a "motor report" to serial out. Tells us if a motor has been calibrated, where it's at, etc.

#### void moveHome()

Method to trigger the "[homing motion](https://www.micropolis.com/support/kb/micropolis-robotics-primer#Homing-routine)" of our robot that "calibrates" the set axis (read that as "...drives the axis towards its limit switch"). Note that the robot will perform this motion at a hard-coded speed, which might be too slow if a joint is under load. And also note that this method *blocks* while it is running.

## RELATED READING

For a broad introduction to the fascinating field of robotics, consider the *[Micropolis Robotics Primer](https://www.micropolis.com/micropolis-robotics-primer)*, part of the Micropolis Handbooks series, Volume 3. You can get it from your local bookseller. ISBN: 978-3-9826166-3-6

## AUTHOR

This code was developed as part of Micropolis' educational robotics efforts.

Micropolis GmbH, [micropolis.com](https://www.micropolis.com/)

## COPYRIGHT & LICENSE

Copyright 2024 Micropolis GmbH. All rights reserved.  

This project is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
