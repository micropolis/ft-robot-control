#include "Motor.h"

Motor::Motor() { }

void Motor::init(int MotorId) {
	_motorId = MotorId;
}

int Motor::getId() {
	return _motorId;
}

bool Motor::moveHome() {

}

