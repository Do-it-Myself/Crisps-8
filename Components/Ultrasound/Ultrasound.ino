#include <HCSR04.h>
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

class Crisps {
public:
  Ultrasound* ultrasoundBlock;
  Ultrasound* ultrasoundTunnel;
  Crisps() = default;
  Crisps(Ultrasound *ultraBlock, Ultrasound *ultraTunnel) {
    ultrasoundBlock = ultraBlock;
    ultrasoundTunnel = ultraTunnel;
  }
  void debug()
  {
    Serial.println("Ultrasound:");
    Serial.println(ultrasoundBlock->dist());
    Serial.println(ultrasoundTunnel->dist());
    delay(2000);
  }
};

Crisps robot;
Ultrasound ultrasound1(4,5);
Ultrasound ultrasound2(6,7);

void setup()
{
  Serial.begin(9600);
  robot = Crisps(&ultrasound1,&ultrasound2);
}

void loop()
{
  robot.debug();
}
