#include <HCSR04.h>
#define distanceFromWall 5

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

  bool distGreaterThan(float distanceCm)
  {
    if (hc.dist() > distanceCm)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  float distError(float distanceCm) { // too far (-ve), too close (+ve)
    return distanceCm - hc.dist();
  }
};

Ultrasound ultrasound(2, 3);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(ultrasound.dist());
  Serial.println(ultrasound.distGreaterThan(distanceFromWall));
}
