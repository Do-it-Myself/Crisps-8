#include <HCSR04.h>
#include <Adafruit_MotorShield.h>

#define TRIG_BLOCK 4
#define ECHO_BLOCK 5
#define AMBER_LIGHT 8
#define RED_LIGHT 11
#define GREEN_LIGHT 12
#define IR_PIN A0

#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 11

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
    Serial.println(analog);
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
    float distance = hc.dist();
    Serial.println(distance);
    return (distance > threshold);
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

 void blockBeforeGrab()
  {
    while (!irBlock.obstacle(IR_THRESHOLD))
    {
    }
  }

bool blockDifferentiate() // detect and grab the block, return whether the block is dense or not
  {
    bool isDense = true;
    if (ultrasoundBlock.denseBlock(DENSE_THRESHOLD))
    { // dense - red
      Serial.println("Dense - red");
      digitalWrite(RED_LIGHT, HIGH);
      digitalWrite(GREEN_LIGHT, LOW);
      delay(5000);
      digitalWrite(RED_LIGHT, LOW);
    }
    else
    { // not dense - green
      Serial.println("Not dense - green");
      digitalWrite(RED_LIGHT, LOW);
      digitalWrite(GREEN_LIGHT, HIGH);
      delay(5000);
      digitalWrite(GREEN_LIGHT, LOW);
      isDense = false;
    }
    return isDense;
  }

void setup() {
  Serial.begin(9600);
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(RED_LIGHT, LOW);
  pinMode(GREEN_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, LOW);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  blockBeforeGrab();
  Serial.println("Block detected!");
  delay(500);
  blockDifferentiate();
}
