#include <Ftduino.h>

class Motor {
  public:
    Motor();
    void init(int motorId, int counterId, int switchId, float pulsesPerDegreeRotation, bool reversed);
    int getId();
    void set(int direction, int speed);
    void set(int direction, int speed, int togo);
    void setDeg(int degrees, int speed);
    unsigned long running();
    void update();
    bool moveHome();				// move motor towards an end stop (limit) switch to calibrate the motor's zero point
    void report();
    int rev(int direction);			// allow turn direction to be reversed. It is not about the shaft direction, but about the resulting geared motion. Also, it is not about clockwise. It is mostly for calibration. Direction LEFT is considered *away* from the one end stop, RIGHT is turning the mechanism towards the end stop limit switch.
    
  private:
	uint8_t _motorId;
	uint8_t _counterId;
	uint8_t _switchId;
	unsigned long _running;
	int _toGo;
	uint8_t _setSpeed;			// pwm value between 0 and 64, via Ftduino::OFF, Ftduino::ON and Ftduino::MAX
	uint8_t _actualSpeed;		// current actual spee, might be modified by ramping
	uint8_t _setDirection;		// pwm value between 0 and 64, via Ftduino::OFF, Ftduino::ON and Ftduino::MAX
	int _timeBudget;
	bool _calibrated;
	int _calibrationPulsesLost;		// due to how we trigger the switch, this number of pulses towards zero are not reachable
	int _absPulses;
	float _pulsesPerDegreeRotation = 63.3;
	bool _reversed;
};

