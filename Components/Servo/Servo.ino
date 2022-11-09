#include <Servo.h>

class Grabber {
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

Grabber grabber(1);

void setup() {
  grabber.begin();
}

void loop() {
  grabber.grab();
  delay(1000);
  grabber.release();
  delay(1000);
}
