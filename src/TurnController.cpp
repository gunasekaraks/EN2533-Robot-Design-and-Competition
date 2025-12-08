#include "TurnController.h"
#include <Arduino.h>

void stopMotorsTC() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void turnAngle(float angleDeg, float wheelDiameterCm, int encoderPPR, float wheelBaseCm, int maxSpeed, int minSpeed) {
  // Normalize angle sign and compute target arc length per wheel
  float absAngle = fabs(angleDeg);
  if (absAngle <= 0.0) return;

  // Wheel geometry
  const float PI_F = 3.14159265f;
  float wheelCircumference = PI_F * wheelDiameterCm; // cm

  // For an in-place rotation each wheel describes radius = wheelBase/2
  float arcLength = (PI_F * wheelBaseCm * absAngle) / 360.0f; // cm per wheel

  long targetPulsesOne = (long)((arcLength / wheelCircumference) * (float)encoderPPR);
  if (targetPulsesOne <= 0) return;
  long targetTotal = 2L * targetPulsesOne;

  // PID tuning (tweak as needed)
  float Kp = 0.40f;
  float Ki = 0.0f;
  float Kd = 0.0f;

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  // Determine turn direction: positive -> right (left forward, right backward)
  bool turnRight = (angleDeg > 0.0f);

  // Reset starting counts
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();

    long rCount = abs(rDelta);
    long lCount = abs(lDelta);
    long currentTotal = rCount + lCount;

    float error = (float)(targetTotal - currentTotal);
    if (error <= 0.0f) break;

    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    // Cap dt to avoid large derivative spikes
    if (dt > 0.2f) dt = 0.2f;

    integral += error * dt;
    // anti-windup
    float maxIntegral = (float)maxSpeed * 2.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    int turnSpeed = (int)output;
    if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
    if (turnSpeed < 0) turnSpeed = 0;
    if (turnSpeed < minSpeed && error > 10.0f) turnSpeed = minSpeed;

    // Apply motor directions depending on requested turn direction
    if (turnRight) {
      // left motor forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, turnSpeed);

      // right motor backward
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, turnSpeed);
    } else {
      // left motor backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, turnSpeed);

      // right motor forward
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, turnSpeed);
    }

    delay(5); // ~200 Hz loop
  }

  stopMotorsTC();
}
