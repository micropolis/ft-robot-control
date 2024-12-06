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
}

int Motor::getId() {
	return _motorId;
}

void Motor::set(int direction, int speed) {
	direction = rev(direction);
	ftduino.motor_set(_motorId, direction, speed);
	_setDirection = direction;
	_setSpeed = speed; // remember for calculateSpeed
	_actualSpeed = speed;
	_running = millis();
	_timeBudget = _setSpeed * 12;
}

void Motor::set(int direction, int speed, int togo) {
	direction = rev(direction);
	ftduino.counter_clear(_counterId);
	ftduino.motor_set(_motorId, direction, speed);
	_setDirection = direction;
	_setSpeed = speed; // remember for calculateSpeed
	_running = millis();
	_toGo = togo;
	_timeBudget = _setSpeed * togo * 1.2;

	Serial.print("M");
	Serial.print(_motorId+1);
	Serial.print(" ");
	Serial.print(direction == 1 ? "LEFT" : "RIGHT");
	Serial.print(" speed:");
	Serial.print(speed);
	Serial.print(" pulses to go:");
	Serial.print(togo);
	Serial.print(" timeBudget:");
	Serial.println(_timeBudget);
}

void Motor::setDeg(int absDegrees, int speed) {
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

	if(roundedToGo > _absPulses){
		int deltaPulses = roundedToGo - _absPulses;
		set(Ftduino::LEFT, speed, deltaPulses);

		Serial.print("M"); Serial.print(_motorId + 1);
		Serial.print(" setDeg:");
		Serial.print(absDegrees);
		Serial.print(" go ");
		Serial.print(deltaPulses);
		Serial.print(" (rounded) pulses ");
		Serial.println(rev(Ftduino::LEFT) == Ftduino::LEFT ? "left": "right");
	}else if(roundedToGo < _absPulses){
		int deltaPulses = _absPulses - roundedToGo;
		set(Ftduino::RIGHT, speed, deltaPulses);

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

unsigned long Motor::running() {
	return _running;
}			

int calculateSpeed(int remaining_pulses, int setSpeed) {
	 // Implement a speed calculation based on remaining pulses
	int minValue = 0;
	int rampedSpeed = map(remaining_pulses, 0, 15, minValue, setSpeed);
	if(rampedSpeed < 15){
		return 15;
	}
	return rampedSpeed;
}

int Motor::rev(int direction) {
	if(_reversed){
		return direction == Ftduino::LEFT ? Ftduino::RIGHT : Ftduino::LEFT;
	}
	return direction;
}

void Motor::update() {
	if(_running){
		if(ftduino.input_get(_switchId)) {
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

					Serial.print("M"); Serial.print(_motorId + 1);
					Serial.print(" pulses remain: ");
					Serial.print( remaining_pulses );
					Serial.print(", after ");
					Serial.print(took);
					Serial.println("ms");
			}else if (remaining_pulses < 15) {
				int rampedSpeed = calculateSpeed(remaining_pulses, _setSpeed);

				if(rampedSpeed != _actualSpeed){
					ftduino.motor_set(_motorId, _setDirection, rampedSpeed);
					_actualSpeed = rampedSpeed;
					Serial.print("M"); Serial.print(_motorId + 1);
					Serial.print(" slow down: ");
					Serial.print( rampedSpeed );
					Serial.print(" at to go: ");
					Serial.println( remaining_pulses );
				}
			}else{
			//	Serial.print("M"); Serial.print(_motorId + 1);
			}
		}

		if (currentMillis - _running >= _timeBudget) {
			ftduino.motor_set(_motorId, Ftduino::BRAKE, Ftduino::MAX);

			if(_running){
				Serial.print("M"); Serial.print(_motorId + 1);
				Serial.print(" failsave timeBudget "); Serial.print(_timeBudget);
				Serial.print(" over, STOPPING at steps: ");
				Serial.println( ftduino.counter_get(_counterId) );
			}else{
			//	Serial.print("M"); Serial.print(_motorId + 1);
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
	int calibrationSpeed = 20; // was Ftduino::MAX/4;
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
		Serial.println(" Error: end stop not reached after 5 secs");
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

