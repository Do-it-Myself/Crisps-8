#include <Adafruit_MotorShield.h>

class Motors {
private:
    int L;
    int R;
    Adafruit_MotorShield MotorShield = Adafruit_MotorShield();

public:
// constructor
    Motors(int L, int R):
    Adafruit_DCMotor *MotorL = MotorShield.getMotor(L);
    Adafruit_DCMotor *MotorR = MotorShield.getMotor(R);
    
// functions
    void begin();
    void forward(int speed);
    void backward(int speed);
    void stop();
    void forwardLeft(int speed);
    void forwardRight(int speed);
};
