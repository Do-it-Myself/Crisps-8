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

Crisps robot;
Ultrasound ultraBlock(TRIG_BLOCK,ECHO_BLOCK);
Ultrasound ultraTunnel(TRIG_TUNNEL,ECHO_TUNNEL);

void setup() {
  Serial.begin(9600);
  robot = Crisps(&ultraBlock,&ultraTunnel);
  robot.button();
}

void loop() {
  robot.task();
}
