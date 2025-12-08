#ifndef TASK4_1_H
#define TASK4_1_H

#include <Arduino.h>

class LineSensors;
class LineFollower;
class UltrasonicSensor;
class CircleFollower;

class Task4_1 {
public:
  // ultrasonicThresholdCm: distance to trigger (default 20 cm)
  Task4_1(LineSensors* qtr, LineFollower* follower, UltrasonicSensor* us, CircleFollower* circle,
          float wheelDiameterCm, int encoderPPR, float wheelBaseCm, int turnMaxSpeed, int turnMinSpeed,
          int ultrasonicThresholdCm = 20);

  void begin();

  // Call repeatedly from loop(); returns true when task completes (circle follower reached INNER_CIRCLE)
  bool update();

private:
  LineSensors* qtr;
  LineFollower* follower;
  UltrasonicSensor* us;
  CircleFollower* circle;

  float wheelDiameterCm;
  int encoderPPR;
  float wheelBaseCm;
  int turnMaxSpeed;
  int turnMinSpeed;

  int ultrasonicThresholdCm;
  bool started;

  enum State { FOLLOW_LINE, TURN_LEFT, RUN_CIRCLE, COMPLETE } state;
};

#endif // TASK4_1_H
