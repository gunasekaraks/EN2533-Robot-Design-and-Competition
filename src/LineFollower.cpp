#include "LineFollower.h"
#include <Arduino.h>

LineFollower::LineFollower(int ena_, int in1_, int in2_, int enb_, int in3_, int in4_, int baseSpeed_, float Kp_, float Ki_, float Kd_)
  : ena(ena_), in1(in1_), in2(in2_), enb(enb_), in3(in3_), in4(in4_), baseSpeed(baseSpeed_), Kp(Kp_), Ki(Ki_), Kd(Kd_), lastError(0), integral(0.0f) {
  pinMode(ena, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
}

void LineFollower::setTunings(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
}

int LineFollower::constrainSpeed(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

void LineFollower::update(unsigned long position, unsigned long centerPosition) {
  long error = (long)centerPosition - (long)position;

  integral += error;
  long derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed  = baseSpeed + (int)correction;
  int rightSpeed = baseSpeed - (int)correction;

  leftSpeed = constrainSpeed(leftSpeed);
  rightSpeed = constrainSpeed(rightSpeed);

  // Set left motor forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, leftSpeed);

  // Set right motor forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, rightSpeed);
}

void LineFollower::stop() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}
