#include <Adafruit_MotorShield.h>
#define SPEED 225
#define TIME 3000

class Motors{
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *Motor;


  // constructor
  Motors(int pin)
  {
    Motor = MotorShield.getMotor(pin);
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
    Motor->setSpeed(150);
    Motor->run(FORWARD);
    Motor->run(RELEASE);

  }

  void forward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(FORWARD);
  }

  void backward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(BACKWARD);
  }

  void stop()
  {
    // make motor stop
    Motor->run(RELEASE);
  }
};

Motors motorL(1);

void setup() {
  Serial.begin(9600);
  motorL.begin();
}

void loop()
{
  Serial.println("Backward");
  motorL.backward(SPEED);
  delay(TIME);
  Serial.println("Stop");
  motorL.stop();
  delay(TIME);
  Serial.println("Forward");
  motorL.forward(SPEED);
  delay(TIME);
}
