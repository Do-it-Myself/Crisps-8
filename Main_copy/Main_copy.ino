#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "HCSR04.h"

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
#define IR_PIN 14

#define MOTOR_L 1
#define MOTOR_R 2

// Tunnel
#define FAST 255
#define WALL_DISTANCE_CM 8.5
#define WALL_DETECTION_CM 12
#define Kp 0.2
#define Ki 0
#define Kd 0
float motorRatio = 1;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError, out;

// Block
#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 10

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

    // turn off light
    digitalWrite(lightPin, LOW);
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

class Crisps
{
public:
  LineFollower lineFollower1;
  LineFollower lineFollower2;
  LineFollower lineFollower3;
  LineFollower lineFollower4;
  Motors motorL;
  Motors motorR;
  IR irBlock;
  Ultrasound ultrasoundBlock;;
  Ultrasound ultrasoundTunnel;
  static const int maxSpeed = 255;
  bool onLine;

  Crisps() = default;
  Crisps(int line0, int line1, int line2, int line3, int motor0, int motor1) : lineFollower1(line0), // this is the line follower on the left for following the main line
                                                                               lineFollower2(line1), // this is the line follower on the right for following the main line
                                                                               lineFollower3(line2),
                                                                               lineFollower4(line3),
                                                                               motorL(motor0, AMBER_LIGHT),
                                                                               motorR(motor1, AMBER_LIGHT),
                                                                               ultrasoundBlock(4,5),
                                                                               ultrasoundTunnel(6,7),
                                                                               irBlock(IR_PIN)
  {
    // Line sensor
    onLine = true;

    // Motor
    motorL.begin();
    motorR.begin();

    // Block light
    pinMode(RED_LIGHT, OUTPUT);
    digitalWrite(RED_LIGHT, LOW);
    pinMode(GREEN_LIGHT, OUTPUT);
    digitalWrite(GREEN_LIGHT, LOW);
  }

  // Motion
  void fullForward()
  {
    motorL.forward(maxSpeed);
    motorR.forward(maxSpeed);
  }

  void fullBackward()
  {
    motorL.backward(maxSpeed);
    motorR.backward(maxSpeed);
  }

  void followLine()
  {
    int step = 1;
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();

    if (!leftLine || !rightLine)
    {
      onLine = false;
      if (!leftLine && rightLine)
      {
        int speedLeft = motorL.getSpeed();
        motorL.forward(speedLeft - step);
      }
      else if (leftLine && !rightLine)
      {
        int speedRight = motorR.getSpeed();
        motorR.forward(speedRight - step);
      }
    }
    else if (onLine == true)
    {
      motorL.forward(maxSpeed);
      motorR.forward(maxSpeed);
    }
  }

  // Tunnel
  bool inTunnel()
  {
    // condi 1 - detect wall
    // condi 2 - cannot detect line
    // condi 1 & 2 to be true
    return (ultrasoundTunnel.wall(WALL_DETECTION_CM) /*&& cannot detect line*/);
  }

  void tunnelPID()
  {
    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime);
    error = (double)ultrasoundTunnel.distError(WALL_DISTANCE_CM);
    cumError += error * elapsedTime;
    rateError = (error - lastError) / elapsedTime;

    out = Kp * error + Ki * cumError + Kd * rateError; // too far (-ve), too close (+ve)
    motorRatio = 1 + out;                              // too far (< 1), too close (> 1)

    lastError = error;
    previousTime = currentTime;

    Serial.print("Error:");
    Serial.println(error);
    Serial.print("Out:");
    Serial.println(out);
    Serial.print("Ratio:");
    Serial.println(motorRatio);
    Serial.println("");

    if (motorRatio > 1)
    {
      motorL.forward(FAST);
      motorR.forward(FAST / motorRatio);
    }
    else if (motorRatio > 0)
    {
      motorL.forward(FAST * motorRatio);
      motorR.forward(FAST);
    }
    else
    {
      motorL.backward(abs(FAST * motorRatio));
      motorR.forward(FAST);
    }
  }

  // Block
  void blockDetection()
  {
    if (irBlock.obstacle(IR_THRESHOLD))
    { // detected obstacle
      if (ultrasoundBlock.denseBlock(DENSE_THRESHOLD))
      { // dense - red
        digitalWrite(RED_LIGHT, HIGH);
        digitalWrite(GREEN_LIGHT, LOW);
        delay(5000);
        digitalWrite(RED_LIGHT, LOW);
      }
      else
      { // not dense - green
        digitalWrite(RED_LIGHT, LOW);
        digitalWrite(GREEN_LIGHT, HIGH);
        delay(5000);
        digitalWrite(GREEN_LIGHT, LOW);
      }
    }
  }

  void debug() {
    Serial.println("Ultrasound:");
    Serial.println(ultrasoundBlock.dist());
    Serial.println(ultrasoundTunnel.dist());
    delay(2000);
  }
};

Crisps robot;

void setup () {
  Serial.begin(9600);
  robot = Crisps(0, 1, 2, 3, 1, 2);  
}

void loop () {
  robot.debug();
}
