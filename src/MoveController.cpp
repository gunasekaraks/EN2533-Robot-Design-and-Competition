#include "../include/MoveController.h"

MoveController::MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                               float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                               int maxSpeed_, int minSpeed_)
  : ENA(ena), IN1(in1), IN2(in2), ENB(enb), IN3(in3), IN4(in4),
    wheelDiameter(wheelDiameterCm), encPPR(encoderPPR), wheelBase(wheelBaseCm),
    maxSpeed(maxSpeed_), minSpeed(minSpeed_), Kp(0.45f), Ki(0.0f), Kd(0.09f) {}

void MoveController::begin() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  // Ensure motors stopped
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void MoveController::setPID(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
}

void MoveController::moveForwardCm(int cm) {
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * wheelDiameter; // cm
  long targetPulses = (long)(( (float)cm / wheelCirc) * (float)encPPR);

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();

    long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;

    float error = (float)(targetPulses - avgPulses);
    if (error <= 0.0f) break;

    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0f;
    lastTime = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;

    integral += error * dt;
    float maxIntegral = (float)maxSpeed * 2.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int forwardSpeed = (int)output;
    if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
    if (forwardSpeed < 0) forwardSpeed = 0;
    if (forwardSpeed < minSpeed && error > 20.0f) forwardSpeed = minSpeed;

    // Left motor forward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, forwardSpeed);

    // Right motor forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, forwardSpeed);

    delay(5);
  }

  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void MoveController::moveBackwardCm(int cm) {
  // Mirror of moveForwardCm but drive motors backward
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * wheelDiameter; // cm
  long targetPulses = (long)(((float)cm / wheelCirc) * (float)encPPR);

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  while (true) {
    noInterrupts();
    long rDelta = abs(rightPulse - startRight);
    long lDelta = abs(leftPulse - startLeft);
    interrupts();

    long avgPulses = (rDelta + lDelta) / 2;

    float error = (float)(targetPulses - avgPulses);
    if (error <= 0.0f) break;

    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0f;
    lastTime = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;

    integral += error * dt;
    float maxIntegral = (float)maxSpeed * 2.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int backwardSpeed = (int)output;
    if (backwardSpeed > maxSpeed) backwardSpeed = maxSpeed;
    if (backwardSpeed < 0) backwardSpeed = 0;
    if (backwardSpeed < minSpeed && error > 20.0f) backwardSpeed = minSpeed;

    // Left motor backward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, backwardSpeed);

    // Right motor backward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, backwardSpeed);

    delay(5);
  }

  // Stop motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
