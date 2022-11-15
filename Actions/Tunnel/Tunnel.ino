#include <HCSR04.h>
#include <Adafruit_MotorShield.h>

#define FAST 255
#define WALL_DISTANCE_CM 5
#define WALL_DETECTION_CM 10
#define Kp 0.2
#define Ki 0
#define Kd 0
float motorRatio = 1;
unsigned long currentTime, previousTime;
double elapsedTime;
double distance, error, lastError, cumError, rateError, out;

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

  bool wall(float distanceCm)
  {
    if (hc.dist() < distanceCm)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  double distError(double distanceCm) { // too far (-ve), too close (+ve)
    distance = hc.dist();
    /*
    Serial.print("DistanceCm:");
    Serial.println(distanceCm);
    Serial.print("Distance:");
    Serial.println(distance);
    */
    return distanceCm - distance;
  }
};

Ultrasound ultrasound(2,3);
Motors motorL(1);
Motors motorR(2);

bool inTunnel() {
  // condi 1 - detect wall
  // condi 2 - cannot detect line
  // condi 1 & 2 to be true 
  return (ultrasound.wall(WALL_DETECTION_CM) /*&& cannot detect line*/)
}

// PID control
// sensor on the left
void tunnelPID() {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  error = (double)ultrasound.distError(WALL_DISTANCE_CM);
  cumError += error*elapsedTime;
  rateError = (error - lastError)/elapsedTime;

  out = Kp*error + Ki*cumError + Kd*rateError;
  motorRatio = 1 + out;

  lastError = error;
  previousTime = currentTime;

  Serial.print("Error:");
  Serial.println(error);
  Serial.print("Out:");
  Serial.println(out);
  Serial.print("Ratio:");
  Serial.println(motorRatio);
  Serial.println("");
  
  if (motorRatio > 1) { 
    motorL.forward(FAST / motorRatio);
    motorR.forward(FAST);
  }
  else if (motorRatio > 0) {
    motorL.forward(FAST);
    motorR.forward(FAST * motorRatio);
  }
  else {
    motorL.forward(FAST);
    motorR.backward(abs(FAST * motorRatio));
  }
}

void setup() {
  Serial.begin(9600);
  motorL.begin();
  motorR.begin();
}

void loop() {
  //tunnelP();

  previousTime = millis();
  lastError = ultrasound.distError(WALL_DISTANCE_CM);
  while (ultrasound.inTunnel()) {
    tunnelPID();
  }
}
