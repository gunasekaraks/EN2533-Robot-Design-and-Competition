#ifndef TASK1_H
#define TASK1_H

#include <Arduino.h>

class LineSensors;
class MotorControl;

enum Direction { NORTH, EAST, SOUTH, WEST };

class Task1 {
public:
  // Constructor: pass sensor and motor references
  Task1(LineSensors* sensors, MotorControl* motors,
        float wheelDiameter, int encoderPPR, float wheelBase);

  // Initialize task
  void begin();

  // Update task (call in loop). Returns true when task is complete.
  bool update();

private:
  LineSensors* sensors;
  MotorControl* motors;
  float wheelDiameter;
  int encoderPPR;
  float wheelBase;

  // Grid state
  int gridMap[10][10];        // 0=empty, 1=obstacle, 2=RED, 3=GREEN, 4=BLUE
  bool knownMap[10][10];      // true if junction has been probed
  int curRow = 1;
  int curCol = 0;
  Direction currentDirection = EAST;
  int junctionCount = 0;
  bool autoUpdatePosition = true;
  bool inDeliveryMode = false;
  bool inAvoidanceMode = false;
  int originalAvoidanceTargetRow = -1;
  int originalAvoidanceTargetCol = -1;
  Direction originalAvoidanceDirection = NORTH;

  // Last detected box color
  int lastDetectedBoxColor = 0;

  // Line following
  const int WHITE_THRESHOLD = 500;
  const int JUNCTION_WHITE_COUNT = 7;
  const int BLACK_THRESHOLD = 700;
  const int CENTER = 3500;
  int baseSpeed = 140;
  bool junctionDetected = false;
  long lastError = 0;
  float integral = 0;

  // Drop locations
  const int DROP_ROW = 9;
  const int BLUE_DROP_COL = 2;
  const int GREEN_DROP_COL = 4;
  const int RED_DROP_COL = 6;

  // Color sensor pins
  const int S0 = 39, S1 = 41, S2 = 37, S3 = 35, SENSOR_OUT = 33;
  const int redPin = 6, greenPin = 4, bluePin = 5;

  // Ultrasonic pins
  const int TRIG_PIN = 32;
  const int ECHO_PIN = 30;

  // Helper methods
  void setMotor(int left, int right);
  void stopMotors();
  void hardLeftTurn90();
  void hardRightTurn90();
  void turnAround180();
  void goDownOneRowFromEnd();
  void moveForward(int acm);
  void moveBackward(int acm);
  void updatePositionAfterForward();
  void lineFollowNormal();
  void obstacleAvoid();
  bool isNextObstacle();
  int detectNextBoxColor();
  int detectNextBoxColorMajority(int samples);
  void pickUpBox();
  void placeBox();
  void deliverBoxAndReturn(int color, int savedRow, int savedCol, Direction savedDir);
  void setColor(int r, int g, int b);
  int readColorAverage(bool s2State, bool s3State);
  String detectColorFromFreq(int r, int g, int b);
  const char* directionToString(Direction dir);
  bool navigateToJunction(int startRow, int startCol, Direction startDir, int targetRow, int targetCol);
  bool findPathBFS(int startRow, int startCol, int targetRow, int targetCol, int path[81][2], int& pathLen);
  int detectBoxAtCurrentPosition();
  // Wrapper for color sensing — returns color code: 0=none, 2=RED,3=GREEN,4=BLUE
  int colorsensor();

  // State machine
  bool taskStarted = false;
  bool taskComplete = false;
};

// External encoder variables (used by Task1)
extern volatile long rightPulse;
extern volatile long leftPulse;

#endif // TASK1_H
