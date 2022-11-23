#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <HCSR04.h>

// Pins
#define LINEFOLLOWER_1 0
#define LINEFOLLOWER_2 1
#define LINEFOLLOWER_3 2
#define LINEFOLLOWER_4 3
#define TRIG_BLOCK 4
#define ECHO_BLOCK 5
#define TRIG_TUNNEL 6
#define ECHO_TUNNEL 7
#define AMBER_LIGHT 8
#define SERVO_1 9
#define SERVO_2 10
#define RED_LIGHT 11
#define GREEN_LIGHT 12
#define IR_PIN 14 // A0
#define BUTTON_PIN 15 // A1

#define MOTOR_L 1
#define MOTOR_R 2

class LineFollower{
  private:
  int inputPin;
  double data;
  public:
  LineFollower() = default;
  LineFollower (int p) {
    inputPin = p;
    pinMode(p, INPUT);
  }
  bool getLineData(){
    data = digitalRead(inputPin);
    if (data == 0.0){
      return true; //black is True
    }else{
      return false; //white is False
    }
  }
};

class Motors{
private:
  int lightPin;
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *Motor;
  int currentSpeed;
  bool goingForward;

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

  void forward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(FORWARD);
    goingForward = true;
    currentSpeed = speed;
    
    // turn on light
    digitalWrite(lightPin, HIGH);
  }

  void backward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(BACKWARD);
    currentSpeed = 0;
    goingForward = false;

    // turn on light
    digitalWrite(lightPin, HIGH);
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

class Grabber{
private:
  int servoPin;

public:
  // class field
  Servo servo;

  // constructor
  Grabber() = default;
  Grabber(int servoNum) {
    if (servoNum == 1) {
      servoPin = 10;
    }
    else {
      servoPin = 9;
    }  
  }

  // functions
  void begin() {
    servo.attach(servoPin);
  }

  void grab() {
    servo.write(0);
  }

  void release() {
    servo.write(90);
  }

};

class IR{
private:
  // class field
  int irPin;

public:
  // constructor
  IR() = default;
  IR(int pin) {
    pinMode(pin, INPUT);
    irPin = pin;
  }

  // functions
  bool obstacle(int threshold) {
    int analog = 0;
    int analog_reading;
    for (int i = 0; i < 10; i++) {
      analog_reading = analogRead(irPin);
      analog += analog_reading;
    }
    analog /= 10;
    return (analog < threshold);
  }
};

class Ultrasound{
public:
  // class field
  HCSR04 hc;

  // constructor
  Ultrasound() = default;
  Ultrasound(int trig, int echo): hc(trig, echo){};

  // functions
  float dist()
  {
    return hc.dist();
  }

  bool wall(float distanceCm)
  {
    return (hc.dist() < distanceCm);
  }
  
  bool denseBlock(int threshold) {
    return (hc.dist() > threshold);
  }

  double distError(double distanceCm) { // too far (-ve), too close (+ve)
    double distance = hc.dist();
    return distanceCm - distance;
  }
};
