#include <Ftduino.h>

#include "Motor.h"

Motor Mot1;
Motor Mot2;
Motor Mot3;

void setup() { 
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	Serial.begin(9600);
	while(!Serial);

	ftduino.init();

	Mot1.init(Ftduino::M1, Ftduino::C1, Ftduino::I1, 5.3333333f, false);
	Mot2.init(Ftduino::M2, Ftduino::C2, Ftduino::I2, 8.2555555f, false);
	Mot3.init(Ftduino::M3, Ftduino::C3, Ftduino::I3, 7.1888888f, false);
}

uint8_t switch2state = 0;
uint8_t switch2stateBefore = 0;

void loop() {
	unsigned long currentMillis = millis();

	Mot1.update();
	Mot2.update();
	Mot3.update();

/*
    switch2state = ftduino.input_get(Ftduino::I2);
    if (switch2state == 1 && switch2stateBefore == 0) { // and it's currently pressed:
      Serial.println("switch2");
      delay(50);
    }
    if (switch2state == 0 && switch2stateBefore == 1) { // and it's currently released:
       Serial.println("switch2 released");
       delay(50);
    }
    switch2stateBefore = switch2state;
*/

	if (Serial.available() > 0) {  // Check if there are any incoming bytes
		char incomingChar = Serial.read();  // Read the incoming byte

		if (incomingChar == 'a') {
			Serial.println("M1 left");
			Mot1.set(Ftduino::LEFT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 's') {
			Serial.println("M1 right");
			Mot1.set(Ftduino::RIGHT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 'd') {
			Serial.println("M2 left");
			Mot2.set(Ftduino::LEFT, Ftduino::MAX, 40);

		} else if (incomingChar == 'f') {
			Serial.println("M2 right");
			Mot2.set(Ftduino::RIGHT, Ftduino::MAX, 40);

		} else if (incomingChar == 'g') {
			Serial.println("M3 left");
			Mot3.set(Ftduino::LEFT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 'h') {
			Serial.println("M3 right");
			Mot3.set(Ftduino::RIGHT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 'b') {
			Mot1.setDeg(3, Ftduino::MAX/2);
			Mot2.setDeg(3, Ftduino::MAX);
			Mot3.setDeg(3, Ftduino::MAX);

		} else if (incomingChar == 'n') {
			Mot1.setDeg(45, Ftduino::MAX/2);
			Mot2.setDeg(45, Ftduino::MAX);
			Mot3.setDeg(45, Ftduino::MAX);

		} else if (incomingChar == 'm') {
			Mot1.setDeg(90, Ftduino::MAX/2);
			Mot2.setDeg(60, Ftduino::MAX);
			Mot3.setDeg(90, Ftduino::MAX);

		} else if (incomingChar == '1') {
			Serial.println("M1 homing");
			Mot1.moveHome();
		} else if (incomingChar == '2') {
			Serial.println("M2 homing");
			Mot2.moveHome();
		} else if (incomingChar == '3') {
			Serial.println("M3 homing");
			Mot3.moveHome();
		} else if (incomingChar == '4') {
			Serial.println("Homing axes 1-3");
			Mot1.moveHome();
			Mot2.moveHome();
			Mot3.moveHome();
		} else if (incomingChar == '5') {
			Mot1.report();
			Mot2.report();
			Mot3.report();

		} else if (incomingChar == '<') {
			Serial.println("EMERGENCY OFF");
			ftduino.motor_set(Ftduino::M1, Ftduino::OFF, Ftduino::MAX);
			ftduino.motor_set(Ftduino::M2, Ftduino::OFF, Ftduino::MAX);
			ftduino.motor_set(Ftduino::M3, Ftduino::OFF, Ftduino::MAX);
		}
	}


    digitalWrite(LED_BUILTIN, LOW);
}

