#ifndef MY_SERVO_H
#define MY_SERVO_H

#include <Arduino.h>
#include <Servo.h>

class MyServo {
  public:
    MyServo();	// Default-Constructor

    void init(uint8_t servoId, int pin);
    int getId();
    void setTrimDeg(int8_t trimDeg);
    int8_t getTrimDeg();
    void setDeg(int degrees);
    void report();

    void attach(int pin);
    void write(int value);
    int read();
    void detach();

  private:
	::Servo arduinoServo;
  uint8_t _servoId = 0;
  int8_t _trimDeg = 0;
  bool _calibrated = false;
};

#endif

