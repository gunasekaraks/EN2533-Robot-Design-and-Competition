#include "Task2_2.h"
#include "LineSensors.h"
#include "MotorControl.h"
#include "MoveController.h"
#include "TurnController.h"

// Encoder pulses are maintained elsewhere (move controller uses them)
extern volatile long rightPulse;
extern volatile long leftPulse;

Task2_2::Task2_2(LineSensors *lineSensors, MotorControl *motors_, MoveController *mover_,
                 int leftIRPin_, int rightIRPin_)
  : qtr(lineSensors), motors(motors_), mover(mover_),
    leftIRPin(leftIRPin_), rightIRPin(rightIRPin_),
    Kp(0.025f), Ki(0.0f), Kd(0.015f), baseSpeed(90), lastError(0), integral(0.0f),
    finalStop(false), moveDone(false), straightCounter(0)
{
  const float wheelDiameter = 6.5f; // matches sketch
  const int encoderPPR = 1650;
  maxStraightPulses = (long)((12.0f / (3.1416f * wheelDiameter)) * encoderPPR);
}

void Task2_2::begin() {
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  finalStop = false;
  moveDone = false;
  straightCounter = 0;
  integral = 0;
  lastError = 0;
}

void Task2_2::goStraight() {
  motors->forward(baseSpeed);
}

void Task2_2::rotateLeftShort() {
  motors->turnLeft(120);
}

void Task2_2::rotateRightShort() {
  motors->turnRight(120);
}

void Task2_2::moveDistanceCm(float cm) {
  // delegate to MoveController for accurate movement
  mover->moveForwardCm(cm);
}

void Task2_2::forwardUntilWhiteIR() {
  while (true) {
    int l = digitalRead(leftIRPin);
    int r = digitalRead(rightIRPin);
    if (l == LOW && r == LOW) {
      motors->stop();
      delay(200);
      break;
    }
    motors->forward(90);
    delay(5);
  }
}

void Task2_2::lineFollowPID(uint16_t *sensorValues) {
  unsigned long position = qtr->readLineWhite(sensorValues);
  long error = 3500 - (long)position; // CENTER constant from sketch
  integral += error;
  long derivative = error - lastError;
  lastError = error;
  float correction = Kp * error + Kd * derivative;
  int leftSpeed = baseSpeed + (int)correction;
  int rightSpeed = baseSpeed - (int)correction;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motors->setLeftMotor(leftSpeed, true);
  motors->setRightMotor(rightSpeed, true);
}

bool Task2_2::update() {
  if (moveDone) return true;

  uint16_t sensorValues[8];
  int leftIR = digitalRead(leftIRPin);
  int rightIR = digitalRead(rightIRPin);
  qtr->readRaw(sensorValues);

  int whiteCount = 0;
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < 300) whiteCount++;
    if (sensorValues[i] > 700) blackCount++;
  }

  // AFTER FINAL WHITE STOP -> directly move forward until white IR (skip 25cm, -5cm, and rotation)
  if (finalStop && !moveDone) {
    motors->stop();
    delay(200);

    // Reset PID state
    qtr->readRaw(sensorValues);
    lastError = 0; integral = 0;

    Serial.println(">> Moving forward until white line detected (Task2_2)...");
    forwardUntilWhiteIR();

    Serial.println(">> White line detected. FINAL STOP.");

    // After task completion: move backward 30 cm and turn right 90°
    Serial.println(">> Moving backward 30cm (Task2_2 completion)...");
    mover->moveBackwardCm(25);
    delay(100);
    Serial.println(">> Turning right 90 degrees...");
    // turnAngle(positive = right)
    turnAngle(90.0f, 6.5f, 1650, 14.25f, 120, 30);

    // After turning right, move forward a short distance to clear the area
    Serial.println(">> Moving forward 5cm after turn...");
    mover->moveForwardCm(5);
    delay(100);

    moveDone = true;
    return true;
  }

  if (moveDone) return true;

  // NORMAL LINE FOLLOWING
  if (leftIR == LOW && rightIR == LOW) {
    // Final white stop detected: stop motors and mark finalStop
    motors->stop();
    finalStop = true;
    Serial.println(">> Final stop detected (Task2_2). Motors stopped.");
    return false;
  }

  if (leftIR == LOW && rightIR == HIGH && whiteCount <= 2) {
    // rotate left until center sensors see white
    while (true) {
      qtr->readRaw(sensorValues);
      int wc = 0;
      if (sensorValues[2] < 300) wc++;
      if (sensorValues[3] < 300) wc++;
      if (sensorValues[4] < 300) wc++;
      if (sensorValues[5] < 300) wc++;
      if (wc >= 2) break;
      rotateLeftShort();
      delay(5);
    }
    motors->stop();
    return false;
  }

  if (rightIR == LOW && leftIR == HIGH && whiteCount <= 2) {
    while (true) {
      qtr->readRaw(sensorValues);
      int wc = 0;
      if (sensorValues[2] < 300) wc++;
      if (sensorValues[3] < 300) wc++;
      if (sensorValues[4] < 300) wc++;
      if (sensorValues[5] < 300) wc++;
      if (wc >= 2) break;
      rotateRightShort();
      delay(5);
    }
    motors->stop();
    return false;
  }

  // GO STRAIGHT / DASH-LINE RECOVERY
  if (blackCount >= 6) {
    if (straightCounter == 0) {
      noInterrupts();
      long startR = rightPulse;
      interrupts();
      straightCounter = -startR; // preserve original algorithm
    }

    goStraight();
    delay(5);

    noInterrupts();
    long rDelta = abs(rightPulse + straightCounter);
    interrupts();

    if (rDelta >= maxStraightPulses) {
      motors->stop();
      delay(100);
      Serial.println(">> Dash line missed: moving backward until white detected...");

      while (true) {
        qtr->readRaw(sensorValues);
        if (sensorValues[2] < 300 || sensorValues[3] < 300 || sensorValues[4] < 300 || sensorValues[5] < 300) break;
        motors->backward(80);
        delay(5);
      }
      motors->stop();
      delay(50);

      // slight left rotation before resuming
      Serial.println(">> Slight left rotation before resuming dash line...");
      motors->setLeftMotor(-100, true); // left wheel backward
      motors->setRightMotor(100, true); // right wheel forward
      delay(250);
      motors->stop();
      delay(50);

      straightCounter = 0;
    }
    return false;
  } else {
    straightCounter = 0;
  }

  // PID control line following
  lineFollowPID(sensorValues);
  delay(5);

  return false;
}
