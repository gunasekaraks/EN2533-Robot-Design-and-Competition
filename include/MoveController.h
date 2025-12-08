#ifndef MOVECONTROLLER_H
#define MOVECONTROLLER_H

#include <Arduino.h>

// These are the encoder pulse counters used by the MoveController implementation.
// Your sketch should define these as volatile long and update them in ISRs.
extern volatile long rightPulse;
extern volatile long leftPulse;

class MoveController {
public:
  // Constructor: motor pins (ENA, IN1, IN2, ENB, IN3, IN4), wheel params and encoder PPR
  MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                 float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                 int maxSpeed = 150, int minSpeed = 50);

  // Configure motor pins (call in setup)
  void begin();

  // Move forward an exact distance in centimeters (blocking)
  void moveForwardCm(int cm);
  // Move backward an exact distance in centimeters (blocking)
  void moveBackwardCm(int cm);

  // Set PID gains (optional)
  void setPID(float Kp, float Ki, float Kd);

private:
  int ENA, IN1, IN2, ENB, IN3, IN4;
  float wheelDiameter;
  int encPPR;
  float wheelBase;
  int maxSpeed;
  int minSpeed;

  // PID gains
  float Kp, Ki, Kd;
};

#endif // MOVECONTROLLER_H
