#ifndef MOTOR_H
#define MOTOR_H

#include <Ftduino.h>

class Motor {
  public:
	Motor();					// Default-Constructor
	void init(int motorId, int counterId, int switchId, float pulsesPerDegreeRotation, bool reversed);
	int getId();
	void setMinSpeed(int minSpeed);
	int  getMinSpeed();
	void setMotionProfile(int motionProfile);
	int  getMotionProfile();
	void set(int direction, int speed, int pulsesToGo, int velocity = 0);
	void setDeg(int degrees, int speed, int velocity = 0);
	bool isCalibrated();
	bool isRunning();
	void update();
	bool moveHome();				// move motor towards an end stop (limit) switch to calibrate the motor's zero point
	void report();
	int rev(int direction);			// allow turn direction to be reversed. It is not about the shaft direction, but about the resulting geared motion. Also, it is not about clockwise. It is mostly for calibration. Direction LEFT is considered *away* from the one end stop, RIGHT is turning the mechanism towards the end stop limit switch.

//	void moveJoints(int deg1, int deg2, int deg3);

  private:
	int speedFromDeltaRamped(int pulsesToGo, int pulsesCounted);
	int velocityFromDeltaRamped(int pulsesToGo, int pulsesCounted);

	float _pulsesPerDegreeRotation = 63.3;	// default red fischertechnik encoder motor
	uint8_t _motorId = 0;
	uint8_t _counterId = 0;
	uint8_t _switchId = 0;
	unsigned long _running = 0;
	int _toGo = 0;
	uint8_t _motionProfile = 0;		// 0=raw, 1=ramped, 2=simple PID, 3=ramped+velocity, 4=cascaded PID
	uint8_t _setSpeed = 0;		// pwm value between 0 and 64, via Ftduino::OFF, Ftduino::ON and Ftduino::MAX
	uint8_t _actualSpeed = 0;		// current actual speed, might be modified by ramping
	uint8_t _minSpeed = 14;		// some axes may require higher speed/torque; never go below this value
	uint8_t _maxSpeed = 64;
	uint8_t _setVelocity = 0;
	float _maxVelocity = 150.0;		// Maximum allowable velocity (can be adjusted dynamically)
	float _minVelocity = 30.0;
	uint8_t _setDirection = 1;		// constants in ftDuino are 1=left, 2=right
	int _timeBudget = 0;
	bool _calibrated = false;
	int _calibrationPulsesLost = 0;	// due to how we trigger the switch, this number of pulses towards zero are not reachable
	int _absPulses = 0;			
	bool _reversed = false;
	bool _cruising = false;

	int speedFromPosPID(int targetPosition, int currentPosition);

	// These need to be "tuned" for your specific motor and application. Start with small values and gradually increase them while testing the motor's response. 
	float Kp = 7.0;			// PID: Proportional gain
						// (with 7, we reach max speed=64 at pulses to go >= ~130) 
						// Increasing Kp should make the system more responsive to large errors. For example:
						// Not reaching high speeds when the error is large suggests that the proportional term (Kp) might be too low.
	float Ki = 1.5;			// PID: Integral gain
						// Adjusting Ki to higher values makes the system as a whole more aggressive. If you notice
						// the system is slow to reach the exact target, you might increase Ki slightly, but be
						// cautious as too high Ki can cause oscillations around the target (motor going back and forth)
	float Kd = 5.0;			// PID: Derivative gain
						// Think of it as a damping factor. Kd applies a correction based on the rate of change of
						// the error. When you experience overshoot or oscillation, raising Kd usually leads to
						// smoother, more accurate and more stable results.

	int lastError = 0;			// PID
	float integral = 0;			// PID
	unsigned long lastTime = 0;		// PID
	float maxIntegral = 100;		// PID: Anti-windup constant; Adjust based on your system
	unsigned long deltaTime = 0;

	float velocityFromPosPID(int setpoint, int currentPosition);
	int speedFromVelocityPID(float desiredVelocity, float currentVelocity);

	float Kp_pos = 0.1, Ki_pos = 0.2, Kd_pos = 0.1; // Position PID gains
	float Kp_vel = 0.5, Ki_vel = 0.0, Kd_vel = 0.0; // Velocity PID gains
	float integral_pos = 0, integral_vel = 0;
	float previousError_pos = 0, previousError_vel = 0;

	int speedFromVelocityProp(int speed, float currentVelocity, float desiredVelocity);

	float Kp_prop = 0.6;			// Proportional gain in speedFromVelocityProp()

	float getVelocity(unsigned long currentTime, int currentPosition);

	const unsigned long _velocityTimeout = 1000;
	static const int bufferSize = 3;        // Size of our sampling window
	double positionBuffer[bufferSize];
	unsigned long timeBuffer[bufferSize];
	int bufferIndex = 0;
	float lastVelocity = 0;
	int lastPosition = 0;		// PID+V
};

