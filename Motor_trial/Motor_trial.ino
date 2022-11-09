#include <Adafruit_MotorShield.h>

class Motors
{
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *MotorL = MotorShield.getMotor(1);
  Adafruit_DCMotor *MotorR = MotorShield.getMotor(2);

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

Motors motors;

void setup()
{
  Serial.begin(9600);
  motors.begin();
}

void loop()
{
  Serial.println("Forward");
  motors.forward(150);
  delay(1000);

  Serial.println("Backward");
  motors.backward(150);
  delay(1000);

  Serial.println("Stop");
  motors.stop();
  delay(1000);

  Serial.println("Forward left");
  motors.forwardLeft(150);
  delay(1000);

  Serial.println("Forward right");
  motors.forwardRight(150);
  delay(1000);
}
