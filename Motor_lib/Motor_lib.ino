#include <Adafruit_MotorShield.h>

class Motors
{
private:
  int L, R;
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();

public:
  // constructor
  Motors(int L, int R) : Adafruit_DCMotor *MotorL = MotorShield.getMotor(L);
  Adafruit_DCMotor *MotorR = MotorShield.getMotor(R);

  // functions
  void begin()
  {
    // Find MotorShield
    if (!MotorShield.begin())
    {
      Serial.println("Could not find Motor Shield. Check wiring.");
      while (1)
        ;
    }
    Serial.println("Motor Shield found.");

    // Initial configuration - turn on
    MotorL->setSpeed(150);
    MotorL->run(FORWARD);
    MotorL->run(RELEASE);

    MotorR->setSpeed(150);
    MotorR->run(FORWARD);
    MotorR->run(RELEASE);
  }

  void forward(int speed)
  {
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
  }

  void backward(int speed)
  {
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed);

    // make motor move backward
    MotorL->run(BACKWARD);
    MotorR->run(BACKWARD);
  }

  void stop()
  {
    // make motor stop
    MotorL->run(RELEASE);
    MotorR->run(RELEASE);
  }

  void forwardLeft(int speed)
  { // for turning left
    // set speed of motor
    MotorL->setSpeed(speed / 2);
    MotorR->setSpeed(speed);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
  }

  void forwardRight(int speed)
  { // for turning right
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed / 2);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
  }
};

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}
