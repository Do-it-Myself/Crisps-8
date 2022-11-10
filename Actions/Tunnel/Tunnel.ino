#include <HCSR04.h>
#include <Adafruit_MotorShield.h>

#define FAST 255
#define SLOW 155 
#define WALL_DISTANCE_CM 10
#define WALL_DISTANCE_TOLERANCE 2

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

class Ultrasound
{
public:
  // class field
  HCSR04 hc;

  // constructor
  Ultrasound(int trig, int echo) : hc(trig, echo){};

  // functions
  float dist()
  {
    return hc.dist();
  }

  bool distGreaterThan(float distanceCm)
  {
    if (hc.dist() > distanceCm)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

Ultrasound ultrasound(2,3);
Motors motorL(1);
Motors motorR(2);

void tunnel() { // sensor on the left
  Serial.println(ultrasound.dist());
  if (ultrasound.distGreaterThan(WALL_DISTANCE_CM + WALL_DISTANCE_TOLERANCE)) { // too right -> turn left
    motorL.forward(FAST);
    motorR.forward(FAST);
    Serial.println("Left");
  }
  else if (!ultrasound.distGreaterThan(WALL_DISTANCE_CM - WALL_DISTANCE_TOLERANCE)) { // too left -> turn right
    motorL.forward(FAST);
    motorR.forward(FAST);
    Serial.println("Right");
  }
  else { // within tolerance -> move straight
    motorL.forward(FAST);
    motorR.forward(FAST);
    Serial.println("Straight");
  }
  delay(200);
}

void setup() {
  Serial.begin(9600);
  motorL.begin();
  motorR.begin();
}

void loop() {
  tunnel();
}
