#include <Adafruit_MotorShield.h>

Adafruit_MotorShield MotorShield = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *MotorL = MotorShield.getMotor(1);
Adafruit_DCMotor *MotorR = MotorShield.getMotor(2);

void forward(int speed) {
  // set speed of motor
  MotorL->setSpeed(speed);
  MotorR->setSpeed(speed);

  // make motor move forward
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void backward(int speed) {
  // set speed of motor
  MotorL->setSpeed(speed);
  MotorR->setSpeed(speed);

  // make motor move backward
  MotorL->run(BACKWARD);
  MotorR->run(BACKWARD);
}

void stop() {
  // make motor stop
  MotorL->run(RELEASE);
  MotorR->run(RELEASE);
}

void forwardLeft(int speed) { // for turning left
  // set speed of motor
  MotorL->setSpeed(speed/2);  
  MotorR->setSpeed(speed);

  // make motor move forward
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void forwardRight(int speed) { // for turning right
  // set speed of motor
  MotorL->setSpeed(speed);
  MotorR->setSpeed(speed/2);

  // make motor move forward
  MotorL->run(FORWARD);
  MotorR->run(FORWARD);
}

void setup() {
  Serial.begin(9600); 

  // Find MotorShield
  if (!MotorShield.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
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

void loop() {
  Serial.println("Forward");
  forward(150);
  delay(1000);

  Serial.println("Backward");
  backward(150);
  delay(1000);

  Serial.println("Stop");
  stop();
  delay(1000);

  Serial.println("Forward left");
  forwardLeft(150);
  delay(1000);

  Serial.println("Forward right");
  forwardRight(150);
  delay(1000);
}
