#include <HCSR04.h>
#include <Adafruit_MotorShield.h>

#define TRIG_BLOCK 4
#define ECHO_BLOCK 5
#define AMBER_LIGHT 8
#define RED_LIGHT 11
#define GREEN_LIGHT 12
#define IR_PIN A0

#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 10

class IR{
private:
  // class field
  int irPin;

public:
  // constructor
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
    return (hc.dist() < distanceCm);
  }
  
  bool denseBlock(int threshold) {
    return (hc.dist() > threshold);
  }

  double distError(double distanceCm) { // too far (-ve), too close (+ve)
    double distance = hc.dist();
    /*
    Serial.print("DistanceCm:");
    Serial.println(distanceCm);
    Serial.print("Distance:");
    Serial.println(distance);
    */
    return distanceCm - distance;
  }
};

IR irBlock(IR_PIN);
Ultrasound ultrasoundBlock(TRIG_BLOCK, ECHO_BLOCK);

void blockDetection() {
  if (irBlock.obstacle(IR_THRESHOLD)) { // detected obstacle
    if (ultrasoundBlock.denseBlock(DENSE_THRESHOLD)) { // dense - red
      digitalWrite(RED_LIGHT, HIGH);
      digitalWrite(GREEN_LIGHT, LOW);
      delay(5000);
      digitalWrite(RED_LIGHT, LOW);
    }
    else { // not dense - green
      digitalWrite(RED_LIGHT, LOW);
      digitalWrite(GREEN_LIGHT, HIGH);
      delay(5000);
      digitalWrite(GREEN_LIGHT, LOW);
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(RED_LIGHT, LOW);
  pinMode(GREEN_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, LOW);
}

void loop() {
  blockDetection();
}
