#include "Class.h"

// Tunnel
#define FAST 255
#define ROTATION_DELAY 700
#define FORWARD_DELAY 500
#define WALL_DISTANCE_CM 8.5
#define WALL_DETECTION_CM 12
#define Kp 0.2
#define Ki 0
#define Kd 0

// Block
#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 11

  enum currenttask{
    lineBeforeBlock,
    lineBlockTunnel,
    lineAfterTunnel,
    blockDensity,
    blockPickup,
    rotateLeft,
    rotateRight,
    tunnel,
    blockDropOff
  };
  enum Block{
    block1,
    block2,
    block3
  };
  
class Crisps
{
public:

  LineFollower lineFollower1; // left
  LineFollower lineFollower2; // right
  LineFollower lineFollower3; // front
  LineFollower lineFollower4; // back
  Motors motorL;
  Motors motorR;
  IR irBlock;
  Ultrasound *ultrasoundBlock;
  Ultrasound *ultrasoundTunnel;
  Grabber grabber;
  static const int maxSpeed = 255;
  bool onLine;
  bool firstRotation = false;
  // tunnel
  float motorRatio = 1;
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error, lastError, cumError, rateError, out;

  // branch counter
  int leftBranch = 0;
  int rightBranch = 0;
  double prevLeftBranchTime = 0;
  double currLeftBranchTime = 0;
  double prevRightBranchTime = 0;
  double currRightBranchTime = 0;
  double branchTimeTol = 2000; // 2 s

  currenttask currentTask;
  Block block;

  Crisps() = default;
  Crisps(Ultrasound *ultraBlock, Ultrasound *ultraTunnel) : lineFollower1(LINEFOLLOWER_1), // this is the line follower on the left for following the main line
                                                            lineFollower2(LINEFOLLOWER_2), // this is the line follower on the right for following the main line
                                                            lineFollower3(LINEFOLLOWER_3),
                                                            lineFollower4(LINEFOLLOWER_4),
                                                            motorL(MOTOR_L, AMBER_LIGHT),
                                                            motorR(MOTOR_R, AMBER_LIGHT),
                                                            irBlock(IR_PIN),
                                                            grabber(SERVO_1)
  {
    // Line sensor
    onLine = true;

    // Motor
    motorL.begin();
    motorR.begin();

    // Grabber
    grabber.begin();

    // Ultrasound
    ultrasoundBlock = ultraBlock;
    ultrasoundTunnel = ultraTunnel;

    // Block light
    pinMode(RED_LIGHT, OUTPUT);
    digitalWrite(RED_LIGHT, LOW);
    pinMode(GREEN_LIGHT, OUTPUT);
    digitalWrite(GREEN_LIGHT, LOW);

    currentTask = lineBeforeBlock;
    block = block1;
    delay(2000);
    fullForward();
    delay(3000);
  }

  // Pure Motion
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

  void stop()
  {
    motorL.stop();
    motorR.stop();

    // turn off light
    digitalWrite(AMBER_LIGHT, LOW);
  }

  void clockwise()
  {
    motorL.forward(maxSpeed);
    motorR.backward(maxSpeed);
  }

  void anticlockwise()
  {
    motorL.backward(maxSpeed);
    motorR.forward(maxSpeed);
  }

  void leftAnchoredAnticlockwise()
  {
    motorL.stop();
    motorR.forward(maxSpeed);
  }

  void leftAnchoredClockwise()
  {
    motorL.stop();
    motorR.backward(maxSpeed);
  }

  void rightAnchoredAnticlockwise() 
  {
    motorL.backward(maxSpeed);
    motorR.stop();
  }

  void rightAnchoredClockwise() 
  {
    motorL.forward(maxSpeed);
    motorR.stop();
  }

  // Line following motion
  void followLine()
  {
    int step = 65;
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    Serial.println(leftLine);
    Serial.println(rightLine);
    Serial.println("-----");
    if (!leftLine || !rightLine)
    {
      onLine = false;
      if (!leftLine && rightLine)
      {
        int speedLeft = motorL.getSpeed();
        if (speedLeft - step < 0)
        {
          motorL.forward(0);
        }
        else
        {
          motorL.forward(speedLeft - step);
        }
        Serial.println("left");
        Serial.println(motorL.getSpeed());
      }
      else if (leftLine && !rightLine)
      {
        int speedRight = motorR.getSpeed();
        if (speedRight - step < 0)
        {
          motorR.forward(0);
        }
        else
        {
          motorR.forward(speedRight - step);
        }
        Serial.println("right");
        Serial.println(motorR.getSpeed());
      }
      else
      {
        motorL.forward(maxSpeed);
        motorR.forward(maxSpeed);
      }
    }
    else if (onLine == false)
    {
      onLine = true;
      motorL.forward(maxSpeed);
      motorR.forward(maxSpeed);
    }
  }

  // Line branch detection boolean for signals
  bool hasLeftBranch()
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (leftLine && veryLeftLine && !rightLine && !veryRightLine);
  }

  bool hasRightBranch()
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (rightLine && veryRightLine && !leftLine && !veryLeftLine);
  }
  bool fullBranch(){
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (!leftLine && !rightLine && !veryLeftLine && !veryRightLine);
  }
  bool allBlack(){
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (leftLine && rightLine && veryLeftLine && veryRightLine);
  }

  void countBranch() // !!!! RESET COUNT TO 0 AFTER EACH LAP
  {
    if (hasLeftBranch())
    {
      currLeftBranchTime = millis();
      if (currLeftBranchTime - prevLeftBranchTime > branchTimeTol && leftBranch < 2)
      { // enough time has passed -> new branch; <2 condition - in case overcount
        leftBranch += 1;
      }
    }
    if (hasRightBranch())
    {
      currRightBranchTime = millis();
      if (currRightBranchTime - prevRightBranchTime > branchTimeTol && rightBranch < 3)
      { // enough time has passed -> new branch; <3 condition - in case overcount
        rightBranch += 1;
      }
    }
    prevRightBranchTime = currRightBranchTime;
  }

  // Branches and zones boolean for signals
  bool reachedGreenZone() // less dense
  {
    return (rightBranch == 1);
  }

  bool reachedStartEndZone()
  {
    return (rightBranch == 2);
  }

  bool reachedRedZone() // more dense
  {
    return (rightBranch == 3);
  }

  bool reachedFirstLeftBranch()
  {
    return (leftBranch == 1);
  }

  bool reachedSecondLeftBranch()
  {
    return (leftBranch == 2);
  }

  // Tunnel - bool might need averaging
  bool inTunnel() // detect when it is inside a tunnel -> trigger tunnelPID
  {
    return (ultrasoundTunnel->wall(WALL_DETECTION_CM));
  }

  void tunnelPID()
  {
    currentTime = millis();
    elapsedTime = (double)(currentTime - previousTime);
    error = (double)ultrasoundTunnel->distError(WALL_DISTANCE_CM);
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

  void triggerTunnelPID() // triggered when inTunnel, break when outTunnel
  {
    while (inTunnel())
    {
      tunnelPID();
    }
  }

  // Block collection
  void blockBeforeGrab()
  {
    // rotate 90 deg anti-clockwise
    leftAnchoredAnticlockwise();
    delay(ROTATION_DELAY); 
    bool rightLine = false;
    do
    { 
      rightLine = lineFollower2.getLineData();
    } while (!rightLine);
    stop();

    // straight till detect block
    fullForward();
    while (!irBlock.obstacle(IR_THRESHOLD))
    {
    }
    stop();
  }

  bool blockDifferentiate() // detect and grab the block, return whether the block is dense or not
  {
    bool isDense = true;
    if (ultrasoundBlock->denseBlock(DENSE_THRESHOLD))
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
      isDense = false;
    }
    // grab foam
    grabber.grab();
    return isDense;
  }

  void blockAfterGrab()
  {
    // rotate 90 deg clockwise
    leftAnchoredClockwise();
    delay(ROTATION_DELAY);
    bool leftLine = false;
    do
    {
      leftLine = lineFollower1.getLineData();
    } while (!leftLine);
    stop();
  }

  // Block release
  void blockBeforeRelease()
  {
    // rotate 90 deg clockwise
    rightAnchoredClockwise();
    delay(ROTATION_DELAY); 
    bool leftLine = false;
    do
    { 
      leftLine = lineFollower1.getLineData();
    } while (!leftLine);
    stop();

    // forward for certain period
    fullForward();
    delay(FORWARD_DELAY);
    stop();
  }

  void blockRelease()
  {
    stop();
    grabber.release();
  }

  void blockAfterRelease()
  {
    // backward for certain period
    fullBackward();
    delay(FORWARD_DELAY);
    stop();

    // rotate 90 deg anticlockwise
    rightAnchoredAnticlockwise();
    delay(ROTATION_DELAY);
    bool rightLine = false;
    do {
      rightLine = lineFollower2.getLineData();
    } while(!rightLine);
    stop();
  }

  // For debugging
  void debug()
  {
    Serial.println(ultrasoundBlock->dist());
    Serial.println(ultrasoundTunnel->dist());
  }

  void start(){
    delay(2000);

  }
  void task(){
    if (block == block1){
      switch (currentTask){
        case lineBeforeBlock:
          if(!firstRotation && fullBranch()){
            //do first rot
            firstRotation = true;
            rightAnchoredClockwise();
            while (!allBlack()){
              delay(5);
            }
          }
          followLine();
          break;
        case lineAfterTunnel:
          break;
        case blockDensity:
          break;
        case blockPickup:
          break;
        case tunnel:
          break;
        case rotateLeft:
          break;
        case rotateRight:
          break;
        case blockDropOff:
          break;
      }
    }
    if (block == block2){
      switch (currentTask){
        case lineBeforeBlock:
          break;
        case lineBlockTunnel:
          break;
        case lineAfterTunnel:
          break;
        case blockDensity:
          break;
        case blockPickup:
          break;
        case tunnel:
          break;
        case rotateLeft:
          break;
        case rotateRight:
          break;
        case blockDropOff:
          break;
      }
    }
    if (block == block3){
      switch (currentTask){
        case lineBeforeBlock:
          break;
        case lineBlockTunnel:
          break;
        case lineAfterTunnel:
          break;
        case blockDensity:
          break;
        case blockPickup:
          break;
        case tunnel:
          break;
        case rotateLeft:
          break;
        case rotateRight:
          break;
        case blockDropOff:
          break;
      }
    }

  }
};