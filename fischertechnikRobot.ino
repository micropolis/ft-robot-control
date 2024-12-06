#include <Ftduino.h>

#include "Motor.h"

Motor Mot1;

void setup() { 
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	Serial.begin(9600);
	while(!Serial);

	ftduino.init();

	ftduino.input_set_mode(Ftduino::I1, Ftduino::SWITCH);
	ftduino.input_set_mode(Ftduino::I2, Ftduino::SWITCH);
	ftduino.input_set_mode(Ftduino::I3, Ftduino::SWITCH);

	Mot1.init(Ftduino::M1);

	ftduino.counter_set_mode(C1, Ftduino::C_EDGE_RISING);
	ftduino.counter_set_mode(C2, Ftduino::C_EDGE_RISING);
	ftduino.counter_set_mode(C3, Ftduino::C_EDGE_RISING);
}

uint8_t switch2state = 0;
uint8_t switch2stateBefore = 0;
uint8_t m1StepPos = 0;

int Mot1running;

void loop() {
	unsigned long currentMillis = millis();

	if(Mot1running){
		if(ftduino.input_get(Ftduino::I1)) {
			digitalWrite(LED_BUILTIN, HIGH);
			ftduino.motor_set(Ftduino::M1, Ftduino::BRAKE, Ftduino::MAX);
			Mot1running = 0;
			Serial.print("M1 limit switch! at steps: ");
			Serial.println(ftduino.counter_get(Ftduino::C1));
		}

		if (currentMillis - Mot1running >= 500) {
			Mot1running = 0;

			ftduino.motor_set(Ftduino::M1, Ftduino::BRAKE, Ftduino::MAX);

			Serial.print("Interval over, M1: steps: ");
			Serial.println(ftduino.counter_get(Ftduino::C1));
		}
	}

	if(ftduino.input_get(Ftduino::I2)) {
		digitalWrite(LED_BUILTIN, HIGH);
		ftduino.motor_set(Ftduino::M2, Ftduino::BRAKE, Ftduino::MAX);
	}
	if(ftduino.input_get(Ftduino::I3)) {
		digitalWrite(LED_BUILTIN, HIGH);
		ftduino.motor_set(Ftduino::M3, Ftduino::BRAKE, Ftduino::MAX);
	}

/*
    switch2state = ftduino.input_get(Ftduino::I2);
    if (switch2state == 1 && switch2stateBefore == 0) { // and it's currently pressed:
      delay(50);
    }
    if (switch2state == 0 && switch2stateBefore == 1) { // and it's currently released:
       delay(50);
    }
    switch2stateBefore = switch2state;
*/

	if (Serial.available() > 0) {  // Check if there are any incoming bytes
		char incomingChar = Serial.read();

		if (incomingChar == 'a') {
			Serial.println("M1 left");
		//	ftduino.motor_counter(Ftduino::M1, Ftduino::LEFT, Ftduino::MAX/2, 30);
			Mot1running = millis();

		} else if (incomingChar == 's') {
			Serial.println("M1 right");
		//	ftduino.motor_counter(Ftduino::M1, Ftduino::RIGHT, Ftduino::MAX/2, 30);
			Mot1running = millis();

		} else if (incomingChar == 'y') {
			Serial.println("M1 left");
		//	ftduino.motor_set(Ftduino::M1, Ftduino::LEFT, Ftduino::MAX/3);
			Mot1running = millis();

		} else if (incomingChar == 'x') {
			Serial.println("M1 left");
		//	ftduino.motor_set(Ftduino::M1, Ftduino::RIGHT, Ftduino::MAX/3);
			Mot1running = millis();

		} else if (incomingChar == 'v') {
			Mot1running = millis();
		}
	}


    digitalWrite(LED_BUILTIN, LOW);
 
}

