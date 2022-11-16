#include <Adafruit_MotorShield.h>
#include <Servo.h>

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
public:
  // class field
  Adafruit_MotorShield MotorShield = Adafruit_MotorShield();
  Adafruit_DCMotor *Motor;
  int currentSpeed;
  bool goingForward;

  // constructor
  Motors() = default;
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
    Motor->run(BACKWARD);
    goingForward = true;
    currentSpeed = speed;
  }

  void backward(int speed)
  {
    // set speed of motor
    Motor->setSpeed(speed);
    Motor->run(FORWARD);
    currentSpeed = 0;
    goingForward = false;
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

class Crisps{
  private:
  LineFollower lineFollower1;
  LineFollower lineFollower2;
  LineFollower lineFollower3;
  LineFollower lineFollower4;
  Motors motorL;
  Motors motorR;
  static const int maxSpeed = 255;
  bool onLine;

  public:
  Crisps() = default;
  Crisps (int line0, int line1, int line2, int line3, int motor0, int motor1): 
    lineFollower1(line0), //this is the line follower on the left for following the main line
    lineFollower2(line1), //this is the line follower on the right for following the main line
    lineFollower3(line2),
    lineFollower4(line3),
    motorL(motor0), 
    motorR(motor1) {
      motorL.begin();
      motorR.begin();
      onLine = true;
    }
  void fullForward(){
    motorL.forward(maxSpeed);
    motorR.forward(maxSpeed);
  }
  void fullBackward(){
    motorL.backward(maxSpeed);
    motorR.backward(maxSpeed);
  }
  void followLine(){
    int step = 1;
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();

    if (!leftLine || !rightLine){
      onLine = false;
      if(!leftLine && rightLine){
        int speedLeft = motorL.getSpeed();
        motorL.forward(speedLeft - step);
      } else if (leftLine && !rightLine) {
        int speedRight = motorR.getSpeed();
        motorR.forward(speedRight - step);    
     }
    } else if (onLine == true){
      motorL.forward(maxSpeed);
      motorR.forward(maxSpeed);
  }
  }
};



Crisps robot;
void setup () {
  Serial.begin(9600);
  Serial.println("Hello setup before!");
  robot = Crisps(0, 1, 2, 3, 1, 2);
  Serial.println("Hello setup after");
  
}

void loop () {
  Serial.println("Full Forward!");
  robot.fullForward();
  delay(1000);
}
