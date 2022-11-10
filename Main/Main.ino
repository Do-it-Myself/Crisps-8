#include <Adafruit_MotorShield.h>
#include <Servo.h>

class LineFollower{
  private:
  static const int threshold = 4;
  int inputPin;
  int analog;
  public:
  LineFollower() = default;
  LineFollower (int p) {
    inputPin = p;
    pinMode(p, INPUT);
  }
  bool getLineData(){
    analog = digitalRead(inputPin);
    if (analog > threshold){
      return true;
    }else{
      return false;
    }
  }
};

class Crisps{
  private:
  LineFollower lineFollowers[4];
  Motors motors[2];
  static const int maxSpeed = 255;

  public:
  Crisps = default();
  Crisps (int line0, int line1, int line2, int line3, int motor0, int motor1){
    lineFollowers[0] = LineFollower(line1); //this is the line follower on the left for following the main line
    lineFollowers[1] = LineFollower(line2); //this is the line follower on the right for following the main line
    lineFollowers[2] = LineFollower(line3);
    lineFollowers[3] = LineFollower(line4);

    motors[0] = Motors(motor0);
    motors[1] = Motors(motor1);

  boolean getLineData(int sensor){
    return lineFollowers[sensor].getLineData();
  }
  void setMotorSpeedForward(int motor, int speed){
    motors[i].forward(speed)
  }
  void setMotorSpeedBackward(int motor, int speed){
    motors[i].backward(speed)
  }
  void fullForward(){
    setMotorSpeedForward(0, maxSpeed);
    setMotorSpeedForward(1, maxSpeed);
  }
  void fullBackward(){
    setMotorSpeedBackward(0, maxSpeed);
    setMotorSpeedBackward(1, maxSpeed);
  }
  void stop(int motor){
    motors[i].stop();
  }
  void followLine(){

  }
  }

};

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
    motor->run(FORWARD)
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

class Grabber{
private:
  int servoPin;

public:
  // class field
  Servo servo;

  // constructor
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

Motors motors(1, 2);
Grabber grabber(1);

void setup () {
  Serial.begin(9600);
  motors.begin();
  grabber.begin();
}

void loop () {

}
