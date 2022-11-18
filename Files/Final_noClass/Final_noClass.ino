#include "Crisps.h"

/* Flow of code
1. Follow line: starting area ->  identify corner then turn right
2. Follow line: detect 1st block (IR + ultrasound) -> determine junction
3. Follow line: detect tunnel (utlrasound and line sensor) -> PID
4. Find line
5. Follow line: reach 1st(red)/3rd(green) junction -> 90 deg clockwise (slowly - in case there's already blocks) 
    -> drop block 
6. Round 2 (think later)
*/

void setup() {
  Serial.begin(9600);

  // Motor
  motorL.begin();
  motorR.begin();

  // Light
  pinMode(RED_LIGHT, OUTPUT);
  digitalWrite(RED_LIGHT, LOW);
  pinMode(AMBER_LIGHT, OUTPUT);
  digitalWrite(AMBER_LIGHT, LOW);
  pinMode(GREEN_LIGHT, OUTPUT);
  digitalWrite(GREEN_LIGHT, LOW);
}

void loop() {
  fullForward();
  Serial.println("Ultrasound:");
  Serial.println(ultrasoundBlock.dist());
  Serial.println(ultrasoundTunnel.dist());
  delay(2000);
}
