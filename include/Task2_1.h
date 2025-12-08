#ifndef TASK2_1_H
#define TASK2_1_H

#include <Arduino.h>

class LineSensors;
class MotorControl;
class MoveController;

// Task 2.1: Line following with IR sensors, encoder-based moves, 180° rotation, and white-line detection
class Task2_1 {
public:
  // Constructor
  // lineSensors: QTR sensor array wrapper
  // motors: MotorControl instance
  // mover: MoveController for precise distance moves
  // leftIRPin, rightIRPin: external IR sensor pins
  Task2_1(LineSensors *lineSensors, MotorControl *motors, MoveController *mover,
          int leftIRPin, int rightIRPin);

  // Initialize (call in setup)
  void begin();

  // Main update: call each loop iteration
  // Returns true if task is complete (final stop reached)
  bool update();

  // Set PID tuning for line following
  void setLinePID(float Kp, float Ki, float Kd);

  // Set line-following base speed
  void setBaseSpeed(int speed);

  // Get current task state
  String getCurrentState() const;

private:
  LineSensors *qtr;
  MotorControl *motors;
  MoveController *mover;
  int leftIRPin, rightIRPin;

  // Task state enum
  enum TaskState {
    LINE_FOLLOW,      // Normal line following with QTR
    FINAL_STOP_WAIT,  // Both IR sensors detected white (exit condition)
    MOVE_25CM,        // Execute 25cm forward
    MOVE_5CM_BACK,    // Execute 5cm backward
    ROTATE_180,       // Rotate 180° left
    FORWARD_UNTIL_WHITE, // Move forward until white line detected by IR
    TASK_COMPLETE     // Task finished
  };

  TaskState state;
  bool taskDone;

  // Line following state
  float Kp, Ki, Kd;
  float integral;
  long lastError;
  int baseSpeed;

  // Straight section tracking
  long straightCounter;
  long maxStraightPulses;
  // (uses global encoder pulses via ISRs in main.cpp)

  // Helper methods
  void lineFollowPID(uint16_t *sensorValues);
  void handleLeftTurn(uint16_t *sensorValues);
  void handleRightTurn(uint16_t *sensorValues);
  void handleIntersection(uint16_t *sensorValues);
  void rotate180Left();
  void forwardUntilWhiteIR();

  // Constants
  static const int CENTER = 3500;
  static const int BLACK_THRESHOLD = 700;
  static const int WHITE_THRESHOLD = 300;
};

#endif // TASK2_1_H
