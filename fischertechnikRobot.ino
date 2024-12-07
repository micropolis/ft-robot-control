#include <Ftduino.h>

#include "Motor.h"

Motor Mot1;
Motor Mot2;
Motor Mot3;

void setup() { 
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	Serial.begin(9600);
	unsigned long startTime = millis();
	while (!Serial && millis() - startTime < 2000) {
		// Wait up to 3 seconds for serial to connect
	}

	ftduino.init();

	Mot1.init(Ftduino::M1, Ftduino::C1, Ftduino::I1, 5.2f, false);
	Mot2.init(Ftduino::M2, Ftduino::C2, Ftduino::I2, 6.8888888f, false); Mot2.setMinSpeed(27);
	Mot3.init(Ftduino::M3, Ftduino::C3, Ftduino::I3, 6.8888888f, false); Mot3.setMinSpeed(20);
	Mot1.setMotionProfile(0);
	Mot2.setMotionProfile(0);
	Mot3.setMotionProfile(0);
}

uint8_t switch2state = 0;
uint8_t switch2stateBefore = 0;

uint8_t step = 0;
uint8_t progress = 0;


void loop() {
	unsigned long currentMillis = millis();

	if(! Mot1.isCalibrated() ){ Mot1.moveHome(); }
	if(! Mot2.isCalibrated() ){ Mot2.moveHome(); }
	if(! Mot3.isCalibrated() ){ Mot3.moveHome(); }


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
	    String inputString = Serial.readStringUntil('\n');
	    int length = inputString.length();

	    if (length >= 3) { // "1l5"
		char incomingChar = inputString.charAt(0);
		char middleChar = inputString.charAt(1);
		String secondString = inputString.substring(2);
		int secondNumber = secondString.toInt();

		if (incomingChar == '1' || incomingChar == '2' || incomingChar == '3') {
			int incomingNumber = incomingChar - '0'; // convert char to int
			Serial.print("M");
			Serial.print(incomingNumber);
			if (middleChar == 'd') {
				Serial.print(" deg ");
				Serial.println(secondNumber);
				Mot1.setMotionProfile(1);
				Mot2.setMotionProfile(1);
				Mot3.setMotionProfile(1);
				if (incomingChar == '1'){		Mot1.setDeg(secondNumber, Ftduino::MAX, 150); }
				else if (incomingChar == '2'){	Mot2.setDeg(secondNumber, Ftduino::MAX, 150); }
				else if (incomingChar == '3'){	Mot3.setDeg(secondNumber, Ftduino::MAX, 150); }
			} else if (middleChar == 'l') {
				Serial.print(" left ");
				Serial.println(secondNumber);
				Mot1.setMotionProfile(0);
				Mot2.setMotionProfile(0);
				Mot3.setMotionProfile(0);
				if (incomingChar == '1'){		Mot1.set(Ftduino::LEFT, Ftduino::MAX/2, secondNumber); }
				else if (incomingChar == '2'){	Mot2.set(Ftduino::LEFT, Ftduino::MAX/2, secondNumber); }
				else if (incomingChar == '3'){	Mot3.set(Ftduino::LEFT, Ftduino::MAX/2, secondNumber); }
			} else if (middleChar == 'r') {
				Mot1.setMotionProfile(0);
				Mot2.setMotionProfile(0);
				Mot3.setMotionProfile(0);
				Serial.print(" right ");
				Serial.println(secondNumber);
				if (incomingChar == '1'){		Mot1.set(Ftduino::RIGHT, Ftduino::MAX/2, secondNumber); }
				else if (incomingChar == '2'){	Mot2.set(Ftduino::RIGHT, Ftduino::MAX/2, secondNumber); }
				else if (incomingChar == '3'){	Mot3.set(Ftduino::RIGHT, Ftduino::MAX/2, secondNumber); }
			}
		}
	    } else if (length == 1) {
		char incomingChar = inputString.charAt(0);

		if (incomingChar == 'a') {
			Serial.println("M1 left");
			Mot1.setMotionProfile(0);
			Mot1.set(Ftduino::LEFT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 's') {
			Serial.println("M1 right");
			Mot1.setMotionProfile(0);
			Mot1.set(Ftduino::RIGHT, Ftduino::MAX/2, 30);

		} else if (incomingChar == 'd') {
			Serial.println("M2 left");
			Mot1.setMotionProfile(0);
			Mot2.set(Ftduino::LEFT, Ftduino::MAX, 40);
		} else if (incomingChar == 'f') {
			Serial.println("M2 right");
			Mot2.setMotionProfile(0);
			Mot2.set(Ftduino::RIGHT, Ftduino::MAX, 40);
		} else if (incomingChar == 'g') {
			Serial.println("M3 left");
			Mot3.setMotionProfile(0);
			Mot3.set(Ftduino::LEFT, Ftduino::MAX/2, 30);
		} else if (incomingChar == 'h') {
			Serial.println("M3 right");
			Mot3.setMotionProfile(0);
			Mot3.set(Ftduino::RIGHT, Ftduino::MAX/2, 30);

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
	}

    digitalWrite(LED_BUILTIN, LOW);
}

