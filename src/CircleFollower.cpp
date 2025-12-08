#include "CircleFollower.h"
#include "ToFSensor.h"
#include "MotorControl.h"

CircleFollower::CircleFollower(ToFSensor *frontLeft, ToFSensor *frontRight, MotorControl *motors_)
  : s1(frontLeft), s2(frontRight), motors(motors_),
    mode(OUTER_CIRCLE), outerTarget(200), innerTarget(150), baseSpeed(120), openingThreshold(300),
    Kp(1.0f), Ki(0.0f), Kd(0.6f), integral(0.0f), lastError(0), lastTime(0),
    openingDetectionCounter(0) {}

void CircleFollower::begin() {
  lastTime = millis();
  mode = OUTER_CIRCLE;
  openingDetectionCounter = 0;
}

void CircleFollower::setOuterTargetDistance(int mm) {
  outerTarget = mm;
}

void CircleFollower::setInnerTargetDistance(int mm) {
  innerTarget = mm;
}

void CircleFollower::setPID(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
}

void CircleFollower::setBaseSpeed(int speed) {
  baseSpeed = speed;
}

void CircleFollower::setOpeningThreshold(int mm) {
  openingThreshold = mm;
}

bool CircleFollower::isOpeningDetected() {
  int d1 = s1 ? s1->readRangeMM() : -1;
  int d2 = s2 ? s2->readRangeMM() : -1;

  // Opening detected if both sensors read > threshold (wall has fallen away)
  if (d1 > openingThreshold && d2 > openingThreshold) {
    return true;
  }
  return false;
}

void CircleFollower::followWall(int targetDistance) {
  int d1 = s1 ? s1->readRangeMM() : -1;
  int d2 = s2 ? s2->readRangeMM() : -1;

  int validCount = 0;
  int sum = 0;
  if (d1 != -1) { sum += d1; validCount++; }
  if (d2 != -1) { sum += d2; validCount++; }

  if (validCount == 0) {
    motors->stop();
    return;
  }

  int avg = sum / validCount;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastTime = now;

  int error = targetDistance - avg;
  integral += error * dt;
  // anti-windup
  if (integral > 1000) integral = 1000;
  if (integral < -1000) integral = -1000;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = constrain(baseSpeed + (int)correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - (int)correction, 0, 255);

  motors->setLeftMotor(leftSpeed, true);
  motors->setRightMotor(rightSpeed, true);
}

FollowMode CircleFollower::update() {
  // Check for mode transitions
  if (mode == OUTER_CIRCLE) {
    // While on outer circle, look for opening
    if (isOpeningDetected()) {
      openingDetectionCounter++;
      if (openingDetectionCounter >= OPENING_CONFIRM_FRAMES) {
        // Confirmed opening found -> transition mode
        mode = TRANSITION;
        Serial.println("Opening detected! Transitioning to INNER_CIRCLE mode...");
        openingDetectionCounter = 0;
      }
    } else {
      openingDetectionCounter = 0; // Reset counter if no opening
    }
    // Continue following outer circle
    followWall(outerTarget);
  }
  else if (mode == TRANSITION) {
    // In transition: continue moving forward into the circle until both sensors see the wall again
    int d1 = s1 ? s1->readRangeMM() : -1;
    int d2 = s2 ? s2->readRangeMM() : -1;

    // If both sensors are now reading reasonable distances (inside the circle), switch to INNER_CIRCLE
    if (d1 > 0 && d1 < openingThreshold && d2 > 0 && d2 < openingThreshold) {
      openingDetectionCounter++;
      if (openingDetectionCounter >= 5) {
        mode = INNER_CIRCLE;
        Serial.println("Inside circle! Now following INNER_CIRCLE...");
        openingDetectionCounter = 0;
      }
    } else {
      openingDetectionCounter = 0;
    }
    // Move forward at base speed (no PID) to pass through opening
    motors->setLeftMotor(baseSpeed, true);
    motors->setRightMotor(baseSpeed, true);
  }
  else if (mode == INNER_CIRCLE) {
    // Follow the inner circle wall at innerTarget distance
    followWall(innerTarget);
  }

  return mode;
}
