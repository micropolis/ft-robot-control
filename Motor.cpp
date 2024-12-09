// encapsulates a fischertechnik encoder-motor with gearbox
#include "Motor.h"

Motor::Motor() { }

void Motor::init(int motorId, int counterId, int switchId, float pulsesPerDegreeRotation, bool reversed) {
	_motorId = motorId;
	_counterId = counterId;
	_switchId = switchId;
	_running = 0;
	_calibrated = false;
	_pulsesPerDegreeRotation = pulsesPerDegreeRotation;
	_reversed = reversed;

	// set input to type "switch": either 1 or 0
	ftduino.input_set_mode(_switchId, Ftduino::SWITCH);

	// set counter input to react on rising edge
	ftduino.counter_set_mode(_counterId, Ftduino::C_EDGE_RISING);


	// in case limit switch is already engaged, move a little away
	if(ftduino.input_get(_switchId)) {
		Serial.println("init: Limit switch engaged: moving away...");

		// drive away from switch and stop as soon as switch opens
		ftduino.motor_set(_motorId, rev(Ftduino::LEFT), Ftduino::MAX/4);

		unsigned long timeBudget = millis() + 5000;
		while(ftduino.input_get(_switchId) && millis() < timeBudget);

		// stop, switch opened or time up
		ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

		if(!ftduino.input_get(_switchId)){
			Serial.println("init: limit switch opened. Okay");
		}else{
			Serial.println("init: Error: disengaging limit switch failed!");
			return false;
		}
	}

	for (int i = 0; i < bufferSize; i++) {
		positionBuffer[i] = 0;
		timeBuffer[i] = 0;
	}
}

int Motor::getId() {
	return _motorId;
}

void Motor::setMinSpeed(int minSpeed) {
	_minSpeed = minSpeed;
}
int Motor::getMinSpeed() {
	return _minSpeed;
}

void Motor::setMotionProfile(int motionProfile) {
	_motionProfile = motionProfile;
}
int Motor::getMotionProfile() {
	return _motionProfile;
}

bool Motor::isCalibrated() {
	return _calibrated;
}

bool Motor::isRunning() {
	return _running;
}

// Turn motor into direction, at given speed, for specified number of pulses
void Motor::set(int direction, int speed, int pulsesToGo, int velocity) {
	direction = rev(direction);
	ftduino.counter_clear(_counterId);
	ftduino.motor_set(_motorId, direction, speed);

	_setDirection = direction;
	_setSpeed = speed;
	_actualSpeed = _setSpeed;
	_toGo = pulsesToGo;
	_setVelocity = velocity;

	_running = millis();
	_timeBudget = _setSpeed * _toGo * 1.2;
	_cruising = false;

	Serial.print("M");
	Serial.print(_motorId+1);
	Serial.print(" ");
	Serial.print(_setDirection == 1 ? "LEFT" : "RIGHT");
	Serial.print(" speed:");
	Serial.print(_setSpeed);
	Serial.print(" pulses to go:");
	Serial.print(_toGo);
	Serial.print(" timeBudget:");
	Serial.print(_timeBudget);
	Serial.print(" mProfile:");
	Serial.println(_motionProfile);
}

void Motor::setDeg(int absDegrees, int speed, int velocity) {
	if(!_calibrated){
		Serial.println("Error: Motor has not been calibrated!");
		return false;
	}

	float pulsesToGo = (absDegrees * _pulsesPerDegreeRotation) - _calibrationPulsesLost;

	if(pulsesToGo < 0){
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.print(" Warning: ");
		Serial.print(absDegrees);
		Serial.print(" outside of reach due to _calibrationPulsesLost. Setting 0 pulses to go.");
		pulsesToGo = 0;
	}

	int roundedToGo = (int)(pulsesToGo + 0.5);

	// Start motor into the right direction at min speed and hand handing motor control over to update()
	int deltaPulses = 0;
	if(roundedToGo > _absPulses){
		deltaPulses = roundedToGo - _absPulses;
		set(Ftduino::LEFT, _minSpeed, deltaPulses, velocity);
	}else if(roundedToGo < _absPulses){
		deltaPulses = _absPulses - roundedToGo;
		set(Ftduino::RIGHT, _minSpeed, deltaPulses, velocity);
	}

	_setSpeed = speed;
	_actualSpeed = _minSpeed;

	if(deltaPulses){
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.print(" setDeg:");
		Serial.print(absDegrees);
		Serial.print(" go ");
		Serial.print(deltaPulses);
		Serial.print(" (rounded) pulses ");
		Serial.println(rev(Ftduino::LEFT) == Ftduino::LEFT ? "left": "right");
	}else{
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.println(" setDeg: 0 to go.");
	}
}


// Returns a speed value based on the delta between how many pulses we need to go and how many we have already travelled (pulses counted)
// This provides a basic "trapezoidal motion profile, divided into three phases: acceleration, cruise and deceleration phase.
int Motor::speedFromDeltaRamped(int pulsesToGo, int pulsesCounted) {
	int pulsesRemaining = pulsesToGo - pulsesCounted;
	int rampedSpeed;

	int minBandingArea = 5;
	int maxBandingArea = 25;
	int bandingArea = constrain((pulsesToGo * maxBandingArea) / 100, minBandingArea, maxBandingArea); 

	_cruising = false;
	if(pulsesCounted < bandingArea){ // ramp up, acceleration phase
		rampedSpeed = map(pulsesCounted, 0, bandingArea, _minSpeed, _setSpeed);
	}else if (pulsesRemaining < bandingArea) { // ramp down, deceleration phase
		rampedSpeed = map(pulsesRemaining, 0, bandingArea, _minSpeed, _actualSpeed); // note _actualSpeed here, as velocity adjustment during cruise may have lowered it and we don't want speed to shoot up only to ramp down
	}else{
		// cruise phase
		rampedSpeed = _setSpeed;
		_cruising = true;
	}

	if(rampedSpeed < _minSpeed){
		return _minSpeed;
	}
	return rampedSpeed;
}

// Unused:
// Returns a velocity value based on the delta between how many pulses we need to go and how many we have already travelled (pulses counted)
// This provides the basis for a basic "trapezoidal motion profile, divided into three phases: acceleration, cruise and deceleration phase.
int Motor::velocityFromDeltaRamped(int pulsesToGo, int pulsesCounted) {
	int pulsesRemaining = pulsesToGo - pulsesCounted;
	int rampedVelocity;

	int minBandingArea = 5;
	int maxBandingArea = 25;
	int bandingArea = constrain((pulsesToGo * maxBandingArea) / 100, minBandingArea, maxBandingArea); 

	_cruising = false;
	if(pulsesCounted < bandingArea){ // ramp up, acceleration phase
		rampedVelocity = map(pulsesCounted, 0, bandingArea, _minVelocity, _setVelocity);
	}else if (pulsesRemaining < bandingArea) { // ramp down, deceleration phase
		rampedVelocity = map(pulsesRemaining, 0, bandingArea, _minVelocity, _setVelocity);
	}else{
		// cruise phase
		rampedVelocity = _setVelocity;
		_cruising = true;
	}

	if(rampedVelocity < _minVelocity){
		return _minVelocity;
	}
	return rampedVelocity;
}


// a PID controller provides a continuous adjustment of motor speed based on the error between the desired and current positions, which, when tuned well,
// provides more accurate positioning and smoother motion compared to a ramping approach as it is more aware of the distance that needs to be travelled
int Motor::speedFromPosPID(int targetPosition, int currentPosition) {
	unsigned long currentTime = millis();
	float deltaTime = currentTime - lastTime; // casting from two unsigned long to one float
	lastTime = currentTime;

	// Calculate error
	int error = targetPosition - currentPosition;

	// Integral term with anti-windup
	integral += Ki * error * deltaTime;
	float minIntegral = 0;
	integral = constrain(integral, minIntegral, maxIntegral);

	// Derivative term
	float derivative = 0;
	if (deltaTime > 0) {  // Avoid division by zero
		derivative = Kd * (error - lastError) / deltaTime;
	}
	lastError = error;

	// Calculate PID output: the Proportional, Integral, and Derivative terms:
	// P: proportional to the current error (P = Kp * error; proportional term)
	// I (integral): sum of all past errors
	// D (derivative): rate of change of error
	float output = Kp * error + integral + derivative;

	// Map PID output to motor speed range
	int speed = map(abs(output), 0, 1000, _minSpeed, _maxSpeed);

	// Ensure speed is within bounds
	speed = constrain(speed, _minSpeed, _maxSpeed);

	// Determine direction
	if (output < 0) {
		speed = -speed;
	}

	return speed;
}

// Unused:
// Velocity PID controller (outer loop)
float Motor::velocityFromPosPID(int targetPosition, int currentPosition) {
	int error = targetPosition - currentPosition;

	// limit integral (anti-windup)
	float maxIntegral = 100.0;
	float minIntegral = 0.0;
	integral_pos += error;
	if (integral_pos > maxIntegral){
		integral_pos = maxIntegral;
	} else if (integral_pos < minIntegral){
		 integral_pos = minIntegral;
	}

	int derivative = (error - previousError_pos) / deltaTime;
	previousError_pos = error;

	float output = (Kp_pos * error)  +  (Ki_pos * integral_pos)  +  (Kd_pos * derivative);

	// Ensure velocity is within bounds
	output = constrain(output, _minVelocity, _maxVelocity);

	return output; // This is the desired velocity
}

// Unused:
// Motor Speed PID controller (inner loop)
int Motor::speedFromVelocityPID(float desiredVelocity, float currentVelocity) {
	float error = desiredVelocity - currentVelocity; // P

	integral_vel += error; // I

	// D
	float derivative = 0;
	if (deltaTime > 0) {  // Avoid division by zero
		float derivative = Kd * (error - lastError) / deltaTime;
	}

	float output = Kp_vel * error + Ki_vel * integral_vel + Kd_vel * derivative;
	previousError_vel = error;

	// Clamp the motor speed to within maxMotorSpeed
	output = constrain(output, _minSpeed, _maxSpeed);

	return output; // This is the motor speed
}

// get a proportionally adjusted speed based on velocity error
int Motor::speedFromVelocityProp(int speed, float currentVelocity, float desiredVelocity) {
	float error = desiredVelocity - currentVelocity;

	float proportional = Kp_prop * error; // calc proportional adjustment value

	// round and cast to int
	if (proportional >= 0) {
		speed += static_cast<int>(proportional + 0.5);
	} else {
		speed += static_cast<int>(proportional - 0.5);
	}

	// Ensure the new speed is within the allowed range
	if (speed < 0) speed = _minSpeed;
	if (speed > 64) speed = _maxSpeed;

	return speed;
}

// Velocity
float Motor::getVelocity(unsigned long currentTime, int currentPosition) {
	float currentVelocity = 0;
	static int numReadings = 0; // Number of valid readings in the buffer

	if (currentPosition != lastPosition) {
		// New reading detected, update buffer
		positionBuffer[bufferIndex] = currentPosition;
		timeBuffer[bufferIndex] = currentTime;

		numReadings = min(numReadings + 1, bufferSize); // Increase readings count until it reaches buffer size

		// Calculate velocity
		int oldestIndex = (bufferIndex + 1) % numReadings; // Ensure we use only valid readings initially
		float positionDifference = positionBuffer[bufferIndex] - positionBuffer[oldestIndex];
		unsigned long timeDiff = timeBuffer[bufferIndex] - timeBuffer[oldestIndex];
		float timeDifference = (float)timeDiff / 1000.0f; // ms to s

		if (numReadings > 1 && timeDifference > 0 && timeDifference < _velocityTimeout / 1000.0f) {
			currentVelocity = positionDifference / timeDifference;
		} else {
			// Optionally handle the error here (e.g., logging)
			Serial.println("Velocity calculation error: Invalid time difference or insufficient readings");
			currentVelocity = 0.00;
		}

		bufferIndex = (bufferIndex + 1) % bufferSize; // Wrap around
		lastVelocity = currentVelocity;
	} else if (currentTime - lastTime > _velocityTimeout) {
		Serial.println("No new readings detected for an extended period");
		lastVelocity = 0;
		currentVelocity = 0;
	} else {
		currentVelocity = lastVelocity;
	}

	lastPosition = currentPosition;
	lastTime = currentTime; // Ensure this is updated with every new reading
	return currentVelocity;
}


int Motor::rev(int direction) {
	if(_reversed){
		return direction == Ftduino::LEFT ? Ftduino::RIGHT : Ftduino::LEFT;
	}
	return direction;
}

void Motor::update() {
	if(_running){
		// Failsafe: Check if axis is triggering its limit switch
		if(ftduino.input_get(_switchId)) {
			// Serial.println("Input 1");
			digitalWrite(LED_BUILTIN, HIGH);
			ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);
			_running = 0;
			Serial.print("M"); Serial.print(_motorId + 1);
			Serial.print(" limit switch! at steps: ");
			Serial.println(ftduino.counter_get(_counterId));

			// drive away from switch and stop as soon as switch opens
			ftduino.motor_set(_motorId, rev(Ftduino::LEFT), Ftduino::MAX/4);

			unsigned long timeBudget = millis() + 5000;
			while(ftduino.input_get(_switchId) && millis() < timeBudget);

			// stop, switch opened or time up
			ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);
		}

		unsigned long currentMillis = millis();
		deltaTime = currentMillis - lastTime; // casting from two unsigned long to one float
		lastTime = currentMillis;

		if(_toGo){
			int remaining_pulses = _toGo - ftduino.counter_get(_counterId);

			if (remaining_pulses <= 0) {
					ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

					if(_calibrated){
						if(_setDirection == rev(Ftduino::LEFT) ){
							_absPulses += ftduino.counter_get(_counterId);
						}else{
							_absPulses -= ftduino.counter_get(_counterId);
						}
					}

					unsigned long took = currentMillis - _running;
					_running = 0;
					_toGo = 0;
			}else{
				if(_motionProfile == 3){
					float currentVelocity = getVelocity(currentMillis, ftduino.counter_get(_counterId) );
					int rampedSpeed = speedFromDeltaRamped(_toGo, ftduino.counter_get(_counterId));

					if(_cruising){
						int adjustedSpeed = speedFromVelocityProp(rampedSpeed, currentVelocity, _setVelocity);

						if(adjustedSpeed != _actualSpeed){
							ftduino.motor_set(_motorId, _setDirection, adjustedSpeed);
							_actualSpeed = adjustedSpeed;
						}
					}else{
						if(rampedSpeed != _actualSpeed){
							ftduino.motor_set(_motorId, _setDirection, rampedSpeed);
							_actualSpeed = rampedSpeed;
						}
					}
				}else if(_motionProfile == 2){
					int pidSpeed = speedFromPosPID(_toGo, ftduino.counter_get(_counterId) );

					if(pidSpeed != _actualSpeed){
						ftduino.motor_set(_motorId, _setDirection, pidSpeed);
						_actualSpeed = pidSpeed;
					}
				}else if(_motionProfile == 1){
					// Ramped motion profile
					int rampedSpeed = speedFromDeltaRamped(_toGo, ftduino.counter_get(_counterId));

					if(rampedSpeed != _actualSpeed){
						ftduino.motor_set(_motorId, _setDirection, rampedSpeed);
						_actualSpeed = rampedSpeed;
				}else{
					// Raw operation: motor is left running as it was started
				}
			}
		}

		// Failsafe: Check if motor is running suspiciously long
		if (currentMillis - _running >= _timeBudget) {
			ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

			if(_running){
				Serial.print("M"); Serial.print(_motorId + 1);
				Serial.print(" failsave timeBudget "); Serial.print(_timeBudget);
				Serial.print(" over, STOPPING at steps: ");
				Serial.println( ftduino.counter_get(_counterId) );
			}else{

			}

			_running = 0;
			_timeBudget = 0;
		}
	}
}

void Motor::report() {
	float absDegrees = (_absPulses + _calibrationPulsesLost) / _pulsesPerDegreeRotation;

	Serial.print("M"); Serial.print(_motorId + 1);
	Serial.print(" cal:");
	Serial.print(_calibrated ? "yes" : "NO");
	Serial.print(" absPu:");
	Serial.print(_absPulses);
	Serial.print(" absDeg:");
	Serial.print(absDegrees, 2); // two decimal places
	Serial.print(" run:");
	Serial.print(_running);
	Serial.print(" toGo:");
	Serial.print(_toGo);
	Serial.print(" timeBud:");
	Serial.print(_timeBudget);
	Serial.println(" ---");
}

bool Motor::moveHome() {
	int calibrationSpeed = Ftduino::MAX/4;
	if(calibrationSpeed < _minSpeed){ calibrationSpeed = _minSpeed; }
	// in case limit switch is already engaged, move a little away
	if(ftduino.input_get(_switchId)) {
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.println(" Limit switch engaged: moving away...");
		ftduino.motor_set(_motorId, rev(Ftduino::LEFT), calibrationSpeed);
		delay(1000);
		ftduino.motor_set(_motorId, Ftduino::BRAKE, 0);
	}

	// check if limit switch open again
	if(ftduino.input_get(_switchId)) {
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.println(" Error: end stop still engaged!");
		return false;
	}

	// drive motor into end stop
	Serial.print("M"); Serial.print(_motorId + 1);
	Serial.println(" moving towards end stop...");
	ftduino.motor_set(_motorId, rev(Ftduino::RIGHT), calibrationSpeed);

	// warte auf Endschalter
	unsigned long timeBudget = millis() + 12000;
	while(!ftduino.input_get(_switchId) && millis() < timeBudget);

	// stop, switch reached or time up
	ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

	if(!ftduino.input_get(_switchId)){
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.println(" Error: end stop not reached after 12 secs");
		return false;
	}

	delay(300);

	// set this position as our reference zero
	_absPulses = 0;
	ftduino.counter_clear(_counterId);

	// drive away from switch and stop as soon as switch opens
	ftduino.motor_set(_motorId, rev(Ftduino::LEFT), calibrationSpeed);

	timeBudget = millis() + 5000;
	while(ftduino.input_get(_switchId) && millis() < timeBudget);

	// stop, switch opened or time up
	ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

	if(ftduino.input_get(_switchId)){
		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.println(" Error: homing failed. Switch still engaged!");
		return false;
	}

	_absPulses = ftduino.counter_get(_counterId);
	_calibrationPulsesLost = _absPulses;
	ftduino.counter_clear(_counterId);
	_calibrated = true;

	Serial.print("M"); Serial.print(_motorId + 1);
	Serial.print(" homing routine successful. ");
	Serial.print("Motor at "); Serial.print(_absPulses); Serial.println(" pulses.");
	return true;
}

