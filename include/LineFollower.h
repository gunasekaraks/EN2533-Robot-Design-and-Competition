#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>

class LineFollower {
public:
  // Constructor takes motor pins and initial PID/baseSpeed settings
  LineFollower(int ena, int in1, int in2, int enb, int in3, int in4, int baseSpeed, float Kp, float Ki, float Kd);

  // Set PID tunings at runtime
  void setTunings(float Kp, float Ki, float Kd);

  // Update controller with the current line position (0..7000 for 8 sensors)
  // centerPosition should typically be 3500 for 8 sensors
  void update(unsigned long position, unsigned long centerPosition = 3500);

  // Emergency stop
  void stop();

private:
  int ena, in1, in2, enb, in3, in4;
  int baseSpeed;
  float Kp, Ki, Kd;
  long lastError;
  float integral;
  int constrainSpeed(int v);
};

#endif // LINEFOLLOWER_H
