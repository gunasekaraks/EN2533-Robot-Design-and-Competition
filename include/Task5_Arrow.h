#ifndef TASK5_ARROW_H
#define TASK5_ARROW_H

#include <Arduino.h>
#include "LineSensors.h"
#include "MotorControl.h"

class Task5_Arrow {
public:
  // Constructor: pass references to sensors and motors
  Task5_Arrow(LineSensors* sensors, MotorControl* motors, 
              int leftIRPin, int rightIRPin,
              float wheelDiameter, int encoderPPR);

  // Initialize task (call in setup)
  void begin();

  // Update task (call in loop). Returns true when task is complete.
  bool update();

private:
  LineSensors* sensors;
  MotorControl* motors;
  int leftIRPin, rightIRPin;
  float wheelDiameter, encoderPPR;

  // PID settings
  float Kp = 0.04f;
  float Ki = 0.0f;
  float Kd = 0.02f;
  int baseSpeed = 90;
  long lastError = 0;
  float integral = 0;

  // Line following parameters
  const int CENTER = 3500;
  const int BLACK_THRESHOLD = 700;
  const int WHITE_THRESHOLD = 300;

  // Motor speed limits
  int maxSpeed = 150;
  int minSpeed = 50;

  // State machine
  bool finalStop = false;
  bool waitMiddleAfterRotation = false;
  bool searchingForLine = true;  // Initial state: move forward until white line detected
  long straightCounter = 0;
  long searchStartTime = 0;
  
  // Encoder tracking (using external rightPulse and leftPulse)
  // These are declared as external in the cpp file
  long maxStraightPulses;

  // Helper methods
  void setMotor(int left, int right);
  void rotateLeft90();
  void moveForwardUntilWhite();  // Move straight with encoders until white line detected
  void rotateLeft();
  void rotateRight();
  void goStraight();
  void stopMotors();
  void moveDistance(float dist_cm);
};

// External encoder variables (used by Task5_Arrow)
extern volatile long rightPulse;
extern volatile long leftPulse;

#endif // TASK5_ARROW_H
