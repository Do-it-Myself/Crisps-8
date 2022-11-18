#include "Class.h"

// Tunnel
#define FAST 255
#define WALL_DISTANCE_CM 8.5
#define WALL_DETECTION_CM 12
#define Kp 0.2
#define Ki 0
#define Kd 0
float motorRatio = 1;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError, out;

// Block
#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 10

LineFollower lineFollower1 = LineFollower(LINEFOLLOWER_1);
LineFollower lineFollower2 = LineFollower(LINEFOLLOWER_2);
LineFollower lineFollower3 = LineFollower(LINEFOLLOWER_3);
LineFollower lineFollower4 = LineFollower(LINEFOLLOWER_4);
Motors motorL = Motors(MOTOR_L, AMBER_LIGHT);
Motors motorR = Motors(MOTOR_R, AMBER_LIGHT);
IR irBlock = IR(IR_PIN);
Ultrasound ultrasoundBlock = Ultrasound(TRIG_BLOCK, ECHO_BLOCK);
Ultrasound ultrasoundTunnel = Ultrasound(TRIG_TUNNEL, ECHO_TUNNEL);
static const int maxSpeed = 255;
bool onLine = true;

// Motion
void fullForward()
{
  motorL.forward(maxSpeed);
  motorR.forward(maxSpeed);
}

void fullBackward()
{
  motorL.backward(maxSpeed);
  motorR.backward(maxSpeed);
}

void followLine()
{
  int step = 1;
  bool leftLine = lineFollower1.getLineData();
  bool rightLine = lineFollower2.getLineData();

  if (!leftLine || !rightLine)
  {
    onLine = false;
    if (!leftLine && rightLine)
    {
      int speedLeft = motorL.getSpeed();
      motorL.forward(speedLeft - step);
    }
    else if (leftLine && !rightLine)
    {
      int speedRight = motorR.getSpeed();
      motorR.forward(speedRight - step);
    }
  }
  else if (onLine == true)
  {
    motorL.forward(maxSpeed);
    motorR.forward(maxSpeed);
  }
}

// Tunnel
bool inTunnel()
{
  // condi 1 - detect wall
  // condi 2 - cannot detect line
  // condi 1 & 2 to be true
  return (ultrasoundTunnel.wall(WALL_DETECTION_CM) /*&& cannot detect line*/);
}

void tunnelPID()
{
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  error = (double)ultrasoundTunnel.distError(WALL_DISTANCE_CM);
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;

  out = Kp * error + Ki * cumError + Kd * rateError; // too far (-ve), too close (+ve)
  motorRatio = 1 + out;                              // too far (< 1), too close (> 1)

  lastError = error;
  previousTime = currentTime;

  Serial.print("Error:");
  Serial.println(error);
  Serial.print("Out:");
  Serial.println(out);
  Serial.print("Ratio:");
  Serial.println(motorRatio);
  Serial.println("");

  if (motorRatio > 1)
  {
    motorL.forward(FAST);
    motorR.forward(FAST / motorRatio);
  }
  else if (motorRatio > 0)
  {
    motorL.forward(FAST * motorRatio);
    motorR.forward(FAST);
  }
  else
  {
    motorL.backward(abs(FAST * motorRatio));
    motorR.forward(FAST);
  }
}

// Block
void blockDetection()
{
  if (irBlock.obstacle(IR_THRESHOLD))
  { // detected obstacle
    if (ultrasoundBlock.denseBlock(DENSE_THRESHOLD))
    { // dense - red
      digitalWrite(RED_LIGHT, HIGH);
      digitalWrite(GREEN_LIGHT, LOW);
      delay(5000);
      digitalWrite(RED_LIGHT, LOW);
    }
    else
    { // not dense - green
      digitalWrite(RED_LIGHT, LOW);
      digitalWrite(GREEN_LIGHT, HIGH);
      delay(5000);
      digitalWrite(GREEN_LIGHT, LOW);
    }
  }
}

void debug()
{
  Serial.println(ultrasoundBlock.dist());
  Serial.println(ultrasoundTunnel.dist());
}
