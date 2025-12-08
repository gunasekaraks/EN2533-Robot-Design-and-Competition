#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H

#include <Arduino.h>

class ToFSensor;
class MotorControl;

class WallFollower {
public:
  // sensors: pointers to two ToFSensor instances (e.g., front-left and front-right)
  // motors: MotorControl pointer to drive motors
  WallFollower(ToFSensor *s1, ToFSensor *s2, MotorControl *motors, int baseSpeed = 120, int targetDistMm = 200);

  // Initialize internal state (call in setup if needed)
  void begin();

  // Set PID params
  void setPID(float Kp, float Ki, float Kd);

  // Call periodically from loop; reads sensors and updates motors
  void update();

  // Expose convenience setters
  void setTargetDistance(int mm);
  void setBaseSpeed(int speed);

private:
  ToFSensor *s1;
  ToFSensor *s2;
  MotorControl *motors;
  int baseSpeed;
  int targetDistance;

  // PID state
  float Kp, Ki, Kd;
  float integral;
  int lastError;
  unsigned long lastTime;
};

#endif // WALLFOLLOWER_H
