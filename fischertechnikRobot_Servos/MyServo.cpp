
#include "MyServo.h"

MyServo::MyServo() { }

void MyServo::init(uint8_t servoId, int pin) {
  	_servoId = servoId;
  	_trimDeg = 0;
  	arduinoServo.attach(pin);
}

int MyServo::getId() {
	return _servoId;
}
void MyServo::setTrimDeg(int8_t trimDeg) {
  _trimDeg = trimDeg;
}
int8_t MyServo::getTrimDeg() {
   return _trimDeg;
}

void MyServo::setDeg(int degrees) {
    arduinoServo.write(degrees + _trimDeg, 128); // go a little slower @ 128
}

void MyServo::report() {
  int degrees = arduinoServo.read();

	Serial.print("S"); Serial.print(_servoId);
	Serial.print(" trim:");
	Serial.print(_trimDeg);
	Serial.print(", at ");
	Serial.print(degrees);
	Serial.println(" degrees");
}

// Methods wrapping the Arduino Servo methods
void MyServo::attach(int pin) {
    arduinoServo.attach(pin);
}
void MyServo::write(int value) {
    arduinoServo.write(value);
}
int MyServo::read() {
    return arduinoServo.read();
}
void MyServo::detach() {
    arduinoServo.detach();
}
