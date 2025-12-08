#ifndef TASK2_2_H
#define TASK2_2_H

#include <Arduino.h>

class LineSensors;
class MotorControl;
class MoveController;

// Task 2.2: Mirror of the provided sketch algorithm (dash-line recovery, IR triggers, encoder moves)
class Task2_2 {
public:
  Task2_2(LineSensors *lineSensors, MotorControl *motors, MoveController *mover,
          int leftIRPin, int rightIRPin);

  // initialize pins/state
  void begin();

  // call in loop(); returns true when task completed
  bool update();

private:
  LineSensors *qtr;
  MotorControl *motors;
  MoveController *mover;
  int leftIRPin, rightIRPin;

  // PID settings (kept for parity with sketch, used in lineFollowPID)
  float Kp, Ki, Kd;
  int baseSpeed;
  long lastError;
  float integral;

  // state
  bool finalStop;
  bool moveDone;

  // straight/dash-line tracking
  long straightCounter;
  long maxStraightPulses;

  void goStraight();
  void rotateLeftShort();
  void rotateRightShort();
  void moveDistanceCm(float cm);
  void forwardUntilWhiteIR();
  void lineFollowPID(uint16_t *sensorValues);
};

#endif // TASK2_2_H
