#include <math.h>

#include "MyServo.h"

MyServo Servo1;
MyServo Servo2;
MyServo Servo3;

  unsigned long previousMillis = 0;
  const long interval = 1000;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	Serial.begin(115200);
	unsigned long startTime = millis();
	while (!Serial && millis() - startTime < 2000) {
		// Wait up to 3 seconds for serial to connect
	}

	Serial.begin(115200);
	Serial.println();
	Serial.println("Servo Control");

	Servo1.init(1, 5); Servo1.setTrimDeg(7);
	Servo2.init(2, 3); Servo1.setTrimDeg(-2);
	Servo3.init(3, 7); Servo1.setTrimDeg(3);

  int pos = 90;
  int del = 12;

  if(0){
    for (int i = 90; i >= 70; i--) {
      pos = i;
      Servo1.setDeg(i);
      delay(del);
    }
    for (int i = 70; i <= 120; i++) {
      pos = i;
      Servo1.setDeg(i);
      delay(del);
    }
    for (int i = 120; i >= 90; i--) {
      pos = i;
      Servo1.setDeg(i);
      delay(del);
    }

    for (int i = 90; i >= 70; i--) {
      pos = i;
      Servo2.setDeg(i);
      delay(del);
    }
    for (int i = 70; i <= 120; i++) {
      pos = i;
      Servo2.setDeg(i);
      delay(del);
    }
    for (int i = 120; i >= 90; i--) {
      pos = i;
      Servo2.setDeg(i);
      delay(del);
    }

    for (int i = 90; i >= 70; i--) {
      pos = i;
      Servo3.setDeg(i);
      delay(del);
    }
    for (int i = 70; i <= 120; i++) {
      pos = i;
      Servo3.setDeg(i);
      delay(del);
    }
    for (int i = 120; i >= 90; i--) {
      pos = i;
      Servo3.setDeg(i);
      delay(del);
    }
  }
}

void loop() {
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
          Serial.print("S");
          Serial.print(incomingNumber);
          if (middleChar == 'd') {
            Serial.print(" deg ");
            Serial.println(secondNumber);

        		if (incomingChar == '1'){		Servo1.setDeg(secondNumber); }
        		else if (incomingChar == '2'){	Servo2.setDeg(secondNumber); }
        		else if (incomingChar == '3'){	Servo3.setDeg(secondNumber); }
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
          }
        }
      }else if (length == 1) { // eg 5 + newline
        char incomingChar = inputString.charAt(0);
        if (incomingChar == '5') {
          Servo1.report();
          Servo2.report();
          Servo3.report();
        }
      }  
    }    

digitalWrite(LED_BUILTIN, LOW);
}
