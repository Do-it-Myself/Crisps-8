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

// IR
bool leftPrevIR[2] = {0, 0};
bool rightPrevIR[2] = {0, 0};
bool veryLeftPrevIR[2] = {0, 0};
bool veryRightPrevIR[2] = {0, 0};

enum currenttask
{
  lineBeforeBlock,
  blockDensity,
  blockPickup,
  lineBlockTunnel,
  tunnel,
  lineAfterTunnel,
  rotateLeft,
  rotateRight,
  blockDropOff
};

enum Block
{
  block1,
  block2,
  block3
};

class Crisps
{
public:
  LineFollower lineFollower1; // left
  LineFollower lineFollower2; // right
  LineFollower lineFollower3; // very left
  LineFollower lineFollower4; // very right
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

  int currentTask;
  int block;
  bool blockIsDense;
  bool blockData_bool = false;
  bool tunnelData_bool = false;

  Crisps() = default;
  Crisps(Ultrasound *ultraBlock, Ultrasound *ultraTunnel) : lineFollower1(LINEFOLLOWER_1), // this is the line follower on the left for following the main line
                                                            lineFollower2(LINEFOLLOWER_2), // this is the line follower on the right for following the main line
                                                            lineFollower3(LINEFOLLOWER_3), // very left
                                                            lineFollower4(LINEFOLLOWER_4), // very right
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

    // Button
    pinMode(BUTTON_PIN, INPUT);

    // Task
    currentTask = 0;
    block = 0;
  }

  // Begin
  void begin()
  {
    Serial.println("Begin");
    fullForward();
    delay(1500);
    Serial.println("Done begin");
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
    ;
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
  bool boolAverage(bool currLine, bool arrLine[3])
  {
    arrLine[0] = arrLine[1];
    arrLine[1] = arrLine[2];
    arrLine[2] = currLine;

    int sum = 0;
    for (int i = 0; i < 3; i++)
    {
      sum += (int)arrLine[i];
    }
    if (sum > 1)
    {
      return 1;
    }
    return 0;
  }

  bool hasLeftBranch()
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (!boolAverage(leftLine, leftPrevIR) && !boolAverage(veryLeftLine, veryLeftPrevIR) && boolAverage(rightLine, rightPrevIR) && boolAverage(veryRightLine, veryRightPrevIR));
  }

  bool hasRightBranch()
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (boolAverage(leftLine, leftPrevIR) && boolAverage(veryLeftLine, veryLeftPrevIR) && !boolAverage(rightLine, rightPrevIR) && !boolAverage(veryRightLine, veryRightPrevIR));
  }

  bool fullBranch()
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    Serial.print("L");
    Serial.println(leftLine);
    Serial.print("R");
    Serial.println(rightLine);
    Serial.print("VL");
    Serial.println(veryLeftLine);
    Serial.print("VR");
    Serial.println(veryRightLine);

    return (!veryLeftLine && !veryRightLine);
  }

  bool fullFirstBranch()
  
  {
    bool leftLine = lineFollower1.getLineData();
    bool rightLine = lineFollower2.getLineData();
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    Serial.print("L");
    Serial.println(leftLine);
    Serial.print("R");
    Serial.println(rightLine);
    Serial.print("VL");
    Serial.println(veryLeftLine);
    Serial.print("VR");
    Serial.println(veryRightLine);

    return (!leftLine || !rightLine || !veryLeftLine || !veryRightLine);
  }

  bool allBlack()
  {
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
  bool blockDetected()
  {
    return irBlock.obstacle(IR_THRESHOLD);
  }

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
    stop();
    bool isDense = true;
    if (ultrasoundBlock->denseBlock(DENSE_THRESHOLD))
    { // dense - red
      digitalWrite(RED_LIGHT, HIGH);
      digitalWrite(GREEN_LIGHT, LOW);
      delay(5000);
      digitalWrite(RED_LIGHT, LOW);
      Serial.println("Dense");
    }
    else
    { // not dense - green
      digitalWrite(RED_LIGHT, LOW);
      digitalWrite(GREEN_LIGHT, HIGH);
      delay(5000);
      digitalWrite(GREEN_LIGHT, LOW);
      isDense = false;
      Serial.println("Not Dense");
    }
    // grab foam
    //grabber.grab();
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
    do
    {
      rightLine = lineFollower2.getLineData();
    } while (!rightLine);
    stop();
  }

  // beginning of program
  void start()
  {
    delay(2000);
  }

  // button to trigger robot begin
  void button()
  {
    while (digitalRead(BUTTON_PIN) == 1)
    {
    }
    delay(1000);
  }

  // data collection
  void dataCollection()
  {
    Serial.println("dataCollection");
    //bool hasLeftBranch_bool = hasLeftBranch(); - DONT UNCOMMENT CUZ THE PROGRAM WILL CRASH
    //bool hasRightBranch_bool = hasRightBranch(); - DONT UNCOMMENT CUZ THE PROGRAM WILL CRASH
    bool blockDetected_bool = blockDetected(); 
    //bool blockDetected_bool = false;
    bool inTunnel_bool = inTunnel();
    //bool fullBranch_bool = fullBranch();
    //bool allBlack_bool = allBlack();
    //countBranch();
    Serial.println(currentTask);
     
    if (!blockData_bool && blockDetected_bool)
    { 
      blockData_bool = true;
      currentTask = blockDensity;
      Serial.println("blockDetected_bool");
    }

    if (!tunnelData_bool && inTunnel_bool) 
    {
      tunnelData_bool = true;
      currentTask = tunnel;
      Serial.println("inTunnel_bool");
    }
    Serial.println(currentTask);
  }

  void task()
  {
    if (block == block1)
    {
      //Serial.println("Before currentTask");
      switch (currentTask)
      {
      case lineBeforeBlock:
        Serial.println("LineBeforeBlock");
        if (!firstRotation && fullFirstBranch())
        {
          // do first rot
          Serial.println("Trigger");
          firstRotation = true;
          rightAnchoredClockwise();
          delay(1000);
          bool leftLine, rightLine;
          do
          {
            leftLine = lineFollower1.getLineData();
            rightLine = lineFollower2.getLineData();
            Serial.print(leftLine);
            Serial.println(rightLine);
            delay(5);
          } while (leftLine && rightLine);
          Serial.println("Triggered");
        }
        followLine();
        break;
      case blockDensity:
        Serial.println("blockDensity");
        blockIsDense = blockDifferentiate();
        currentTask = blockPickup;
        break;
      case blockPickup:
        Serial.println("blockPickup");
        fullForward();
        delay(10);
        followLine();
        currentTask = lineBlockTunnel;
        break;
      case lineBlockTunnel:
        Serial.println("lineBlockTunnel");
        followLine();
        break;
      case tunnel:
        Serial.println("lineBlockTunnel");
        triggerTunnelPID();
        currentTask = lineAfterTunnel;
        break;
      case lineAfterTunnel:
        Serial.println("lineAfterTunnel");
        followLine();
        break;
      case rotateLeft:
        break;
      case rotateRight:
        break;
      case blockDropOff:
        break;
      }
    }
    if (block == block2)
    {
      switch (currentTask)
      {
      case lineBeforeBlock:
        break;
      case blockDensity:
        break;
      case blockPickup:
        break;
      case lineBlockTunnel:
        break;
      case tunnel:
        break;
      case lineAfterTunnel:
        break;
      case rotateLeft:
        break;
      case rotateRight:
        break;
      case blockDropOff:
        break;
      }
    }
    if (block == block3)
    {
      switch (currentTask)
      {
      case lineBeforeBlock:
        break;
      case blockDensity:
        break;
      case blockPickup:
        break;
      case lineBlockTunnel:
        break;
      case tunnel:
        break;
      case lineAfterTunnel:
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