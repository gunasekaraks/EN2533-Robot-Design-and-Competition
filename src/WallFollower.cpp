#include "WallFollower.h"
#include "ToFSensor.h"
#include "MotorControl.h"

WallFollower::WallFollower(ToFSensor *s1_, ToFSensor *s2_, MotorControl *motors_, int baseSpeed_, int targetDistMm)
  : s1(s1_), s2(s2_), motors(motors_), baseSpeed(baseSpeed_), targetDistance(targetDistMm),
    Kp(1.0f), Ki(0.0f), Kd(0.6f), integral(0.0f), lastError(0), lastTime(0) {}

void WallFollower::begin() {
  lastTime = millis();
}

void WallFollower::setPID(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
}

void WallFollower::setTargetDistance(int mm) {
  targetDistance = mm;
}

void WallFollower::setBaseSpeed(int speed) {
  baseSpeed = speed;
}

void WallFollower::update() {
  int d1 = s1 ? s1->readRangeMM() : -1;
  int d2 = s2 ? s2->readRangeMM() : -1;

  int validCount = 0;
  int sum = 0;
  if (d1 != -1) { sum += d1; validCount++; }
  if (d2 != -1) { sum += d2; validCount++; }

  if (validCount == 0) {
    // No valid readings -> stop
    motors->stop();
    return;
  }

  int avg = sum / validCount;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastTime = now;

  int error = targetDistance - avg; // positive -> too close? depends on sign, keep as-is
  integral += error * dt;
  // anti-windup
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = constrain(baseSpeed + (int)correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - (int)correction, 0, 255);

  // Drive forward using MotorControl API
  motors->setLeftMotor(leftSpeed, true);
  motors->setRightMotor(rightSpeed, true);
}
