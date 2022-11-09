#include "motors.h"

void Motors::begin() {
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

void Motors::forward(int speed) {
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
}

void Motors::backward(int speed) {
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed);

    // make motor move backward
    MotorL->run(BACKWARD);
    MotorR->run(BACKWARD);
}

void Motors::stop() {
    // make motor stop
    MotorL->run(RELEASE);
    MotorR->run(RELEASE);
}

void Motors::forwardLeft(int speed) {
    // set speed of motor
    MotorL->setSpeed(speed/2);  
    MotorR->setSpeed(speed);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
}

void Motors::forwardRight(int speed) {
    // set speed of motor
    MotorL->setSpeed(speed);
    MotorR->setSpeed(speed/2);

    // make motor move forward
    MotorL->run(FORWARD);
    MotorR->run(FORWARD);
}
