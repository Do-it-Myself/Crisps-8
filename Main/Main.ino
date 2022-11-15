#include <Adafruit_MotorShield.h>
#include <Servo.h>

class LineFollower{
  private:
  int inputPin;
  double data;
  public:
  LineFollower();
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
  Motors();
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
  Grabber();
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
  LineFollower lineFollowers[4];
  Motors motors[2];
  static const int maxSpeed = 255;
  bool onLine;

  public:
  Crisps();
  Crisps (int line0, int line1, int line2, int line3, int motor0, int motor1){
    lineFollowers[0] = LineFollower(line0); //this is the line follower on the left for following the main line
    lineFollowers[1] = LineFollower(line1); //this is the line follower on the right for following the main line
    lineFollowers[2] = LineFollower(line2);
    lineFollowers[3] = LineFollower(line3);

    motors[0] = Motors(motor0); //left motor
    motors[1] = Motors(motor1); //right motor

    motors[0].begin();
    motors[1].begin();

    onLine = true;
  }
  bool getLineData(int sensor){
    return lineFollowers[sensor].getLineData();
  }
  void setMotorSpeedForward(int i, int speed){
    motors[i].forward(speed);
  }
  void setMotorSpeedBackward(int i, int speed){
    motors[i].backward(speed);
  }
  void fullForward(){
    setMotorSpeedForward(0, maxSpeed);
    setMotorSpeedForward(1, maxSpeed);
  }
  void fullBackward(){
    setMotorSpeedBackward(0, maxSpeed);
    setMotorSpeedBackward(1, maxSpeed);
  }
  void stop(int i ){
    motors[i].stop();
  }
  void followLine(){
    int step = 1;
    bool leftLine = getLineData(0);
    bool rightLine = getLineData(1);

    if (!leftLine || !rightLine){
      onLine = false;
      if(!leftLine && rightLine){
        int speedLeft = motors[0].getSpeed();
        motors[0].forward(speedLeft - step);
      } else if (leftLine && !rightLine) {
        int speedRight = motors[1].getSpeed();
        motors[1].forward(speedRight - step);    
     }
    } else if (onLine == true){
      motors[0].forward(maxSpeed);
      motors[1].forward(maxSpeed);
  }
  }
};



Crisps robot;
void setup () {
  Serial.begin(9600);
  robot = Crisps(0, 1, 2, 3, 1, 2);
  
}

void loop () {
  robot.followLine();
}