#include "Class.h"

// Tunnel
#define FAST 255
#define WALL_DISTANCE_CM 8.5
#define WALL_DETECTION_CM 12
#define Kp 0.2
#define Ki 0
#define Kd 0

// Block
#define IR_THRESHOLD 780
#define DENSE_THRESHOLD 10

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

  // Line branch detection
  bool hasLeftBranch()
  {
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (veryLeftLine && !veryRightLine);
  }

  bool hasRightBranch()
  {
    bool veryLeftLine = lineFollower3.getLineData();
    bool veryRightLine = lineFollower4.getLineData();

    return (veryRightLine && !veryLeftLine);
  }

  void countBranch()
  {
    if (hasLeftBranch())
    {
      currLeftBranchTime = millis();
      rightBranch = 0; // reset rightBranch as we passed through all leftBranch already
      if (currLeftBranchTime - prevLeftBranchTime > branchTimeTol && leftBranch < 2)
      { // enough time has passed -> new branch; <2 condition - in case overcount
        leftBranch += 1;
      }
    }
    if (hasRightBranch())
    {
      currRightBranchTime = millis();
      leftBranch = 0; // reset leftBranch as we passed through all leftBranch already
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
    while (!outTunnel())
    {
      tunnelPID();
    }
  }

  // Block collection
  void blockBeforeGrab()
  {
    // rotate 90 deg anti-clockwise
    anticlockwise();
    delay(700); // till front has left the main line
    bool frontLine = false;
    do
    { // front hasn't detected branches yet
      frontLine = lineFollower3.getLineData();
    } while (!frontLine);
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
    // backward for short time (experiment ?)
    fullBackward();
    delay(700);
    stop();

    // rotate 90 deg clockwise
    clockwise();
    delay(700); // till front has left the main line
    bool frontLine = false;
    do
    { // front hasn't detected branches yet
      frontLine = lineFollower3.getLineData();
    } while (!frontLine);
    stop();
  }

  // Block release
  void blockBeforeRelease() {
    // rotate 90 deg clockwise
    clockwise();
    delay(700); // till front has left the main line
    bool frontLine = false;
    do
    { // front hasn't detected branches yet
      frontLine = lineFollower3.getLineData();
    } while (!frontLine);
    stop();

    // forward for certain period
    fullForward();
    delay(1000);
    stop();
  }

  void blockRelease()
  {
    stop();
    grabber.release();
  }

  void blockAfterRelease() {
    // backward for certain period

    // rotate 90 deg anticlockwise
  }

  // For debugging
  void debug()
  {
    Serial.println(ultrasoundBlock->dist());
    Serial.println(ultrasoundTunnel->dist());
  }
};