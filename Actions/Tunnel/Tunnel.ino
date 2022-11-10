#include <HCSR04.h>

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
};

Ultrasound ultrasound(2,3);

void tunnel() {
  
}


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
