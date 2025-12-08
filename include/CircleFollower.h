#ifndef CIRCLEFOLLOWER_H
#define CIRCLEFOLLOWER_H

#include <Arduino.h>

class ToFSensor;
class MotorControl;

// Enum for current following mode
enum FollowMode {
  OUTER_CIRCLE,    // Following the outer perimeter of the circle
  TRANSITION,      // Moving through the opening to enter the circle
  INNER_CIRCLE     // Following the inner perimeter of the circle
};

class CircleFollower {
public:
  // Constructor: two ToF sensors and MotorControl instance
  CircleFollower(ToFSensor *frontLeft, ToFSensor *frontRight, MotorControl *motors);

  // Initialize (call in setup)
  void begin();

  // Set target distance for wall following (outer circle typically 200-250 mm, inner may differ)
  void setOuterTargetDistance(int mm);
  void setInnerTargetDistance(int mm);

  // Set PID gains
  void setPID(float Kp, float Ki, float Kd);

  // Set base speed for the robot
  void setBaseSpeed(int speed);

  // Set opening detection threshold: if both sensors read > openingThresholdMm, it's an opening
  void setOpeningThreshold(int mm);

  // Main update function: call this each loop iteration
  // Returns current mode (OUTER_CIRCLE, TRANSITION, or INNER_CIRCLE)
  FollowMode update();

  // Get current mode
  FollowMode getCurrentMode() const { return mode; }

private:
  ToFSensor *s1;  // front-left sensor
  ToFSensor *s2;  // front-right sensor
  MotorControl *motors;

  FollowMode mode;
  int outerTarget;
  int innerTarget;
  int baseSpeed;
  int openingThreshold;

  // PID state
  float Kp, Ki, Kd;
  float integral;
  int lastError;
  unsigned long lastTime;

  // Opening detection: count consecutive frames where both sensors see an opening
  int openingDetectionCounter;
  static const int OPENING_CONFIRM_FRAMES = 10; // Require 10 frames to confirm opening

  // Helper: perform PID-based wall following at given target distance
  void followWall(int targetDistance);

  // Helper: detect if there's an opening (both sensors reading high)
  bool isOpeningDetected();
};

#endif // CIRCLEFOLLOWER_H
