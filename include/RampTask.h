#ifndef RAMP_TASK_H
#define RAMP_TASK_H

#include <Arduino.h>

class LineSensors;
class MotorControl;
class MoveController;

// Simple Ramp task placeholder. Implement ramp-specific behavior in update().
class RampTask {
public:
  RampTask(LineSensors* qtr, MotorControl* motors, MoveController* mover, int leftIR, int rightIR);
  void begin();
  // Call each loop iteration. Return true when task complete.
  bool update();

private:
  LineSensors* qtr;
  MotorControl* motors;
  MoveController* mover;
  int leftIRPin;
  int rightIRPin;
  bool started;
};

#endif // RAMP_TASK_H
