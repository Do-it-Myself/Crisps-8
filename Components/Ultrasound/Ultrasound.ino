#include "HCSR04.h"
#define distanceFromWall 5

class Ultrasound
{
public:
  // class field
  HCSR04 hc;

  // constructor
  Ultrasound() = default;
  Ultrasound(int trig, int echo) : hc(trig, echo){};

  // functions
  float dist()
  {
    return hc.dist();
  }

  bool wall(float distanceCm)
  {
    if (hc.dist() < distanceCm)
    {
      return true;
    }
    else
    {
      return false;
    }
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

Ultrasound ultrasound1;
Ultrasound ultrasound2;

void setup()
{
  Serial.begin(9600);
  ultrasound1 = Ultrasound(4,5);
  ultrasound2 = Ultrasound(6,7);
}

void loop()
{
  Serial.println("Ultrasound:");
  Serial.println(ultrasound1.dist());
  Serial.println(ultrasound2.dist());
  delay(2000);
}
