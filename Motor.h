#include <Ftduino.h>

class Motor {
  public: 
    Motor();				// default constructor
    void init(int MotorId);
    int getId();
    bool moveHome();

  private:
	int _motorId;
};

