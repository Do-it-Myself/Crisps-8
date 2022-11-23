#include <Adafruit_MotorShield.h>
#define SLOW 100.1
#define FAST 255
#define TIME 10000
#define AMBER_LIGHT 8

class Motors{
private:
  int lightPin;
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *Motor;
  int currentSpeed;
  bool goingForward;
  double prevFlash = 0;
  double currFlash;

  // constructor
  Motors() = default;
  Motors(int pin, int light)
  {
    Motor = MotorShield.getMotor(pin);
    lightPin = light;
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

    // configure amber light
    pinMode(lightPin, OUTPUT);
    digitalWrite(lightPin, LOW);
  }

  void flash() 
  {
    currFlash = millis();
    Serial.print("prev:");
    Serial.println(prevFlash);
    Serial.print("curr:");
    Serial.println(currFlash);
    if (currFlash - prevFlash > 1000) {
      // flash light
      digitalWrite(lightPin, !digitalRead(lightPin));

      // reset prevFlash
      prevFlash = currFlash;
    }
  }

  void forward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(FORWARD);
    goingForward = true;
    currentSpeed = speed;
    
    // flash light
    flash();
  }

  void backward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(BACKWARD);
    currentSpeed = 0;
    goingForward = false;

    // turn on light
    flash();
  }

  void stop()
  {
    // make motor stop
    Motor->run(RELEASE);
    currentSpeed = 0;
  }
  
  int getSpeed()
  {
    return currentSpeed;
  }
};

Motors motorL(1, AMBER_LIGHT);
Motors motorR(2, AMBER_LIGHT);

void setup()
{
  Serial.begin(9600);
  motorL.begin();
  motorR.begin();
}

void loop()
{
  motorR.backward(FAST);
}
