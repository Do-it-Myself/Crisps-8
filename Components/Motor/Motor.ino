#include <Adafruit_MotorShield.h>
#define SPEED 225
#define TIME 3000

class Motors
{
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *MotorL;
  Adafruit_DCMotor *MotorR;

  // constructor
  Motors(int L, int R)
  {
    MotorL = MotorShield.getMotor(L);
    MotorR = MotorShield.getMotor(R);
  }

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

Motors motors(1, 2);

void setup()
{
  Serial.begin(9600);
  motors.begin();
}

void loop()
{
  Serial.println("Forward");
  motors.forward(SPEED);
  delay(TIME);

  Serial.println("Backward");
  motors.backward(SPEED);
  delay(TIME);

  Serial.println("Stop");
  motors.stop();
  delay(TIME);

  Serial.println("Forward left");
  motors.forwardLeft(SPEED);
  delay(TIME);

  Serial.println("Forward right");
  motors.forwardRight(SPEED);
  delay(TIME);
}
