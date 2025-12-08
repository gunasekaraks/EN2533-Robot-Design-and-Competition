#include "Task2_1.h"
#include "LineSensors.h"
#include "MotorControl.h"
#include "MoveController.h"
#include "TurnController.h"

// Encoder pulses updated by ISRs in main.cpp
extern volatile long rightPulse;
extern volatile long leftPulse;

Task2_1::Task2_1(LineSensors *lineSensors, MotorControl *motors_, MoveController *mover_,
                 int leftIRPin_, int rightIRPin_)
  : qtr(lineSensors), motors(motors_), mover(mover_),
    leftIRPin(leftIRPin_), rightIRPin(rightIRPin_),
    state(LINE_FOLLOW), taskDone(false),
    Kp(0.035f), Ki(0.0f), Kd(0.015f), integral(0.0f), lastError(0), baseSpeed(90),
    straightCounter(0) {
  
  const float wheelDiameter = 6.5;  // cm (must match robot)
  const int encoderPPR = 1650;
  maxStraightPulses = (long)((20.0 / (3.1416f * wheelDiameter)) * encoderPPR);
}

void Task2_1::begin() {
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  state = LINE_FOLLOW;
  taskDone = false;
  integral = 0;
  lastError = 0;
  straightCounter = 0;
}

void Task2_1::setLinePID(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_; Ki = Ki_; Kd = Kd_;
}

void Task2_1::setBaseSpeed(int speed) {
  baseSpeed = speed;
}

String Task2_1::getCurrentState() const {
  switch (state) {
    case LINE_FOLLOW: return "LINE_FOLLOW";
    case FINAL_STOP_WAIT: return "FINAL_STOP_WAIT";
    case MOVE_25CM: return "MOVE_25CM";
    case MOVE_5CM_BACK: return "MOVE_5CM_BACK";
    case ROTATE_180: return "ROTATE_180";
    case FORWARD_UNTIL_WHITE: return "FORWARD_UNTIL_WHITE";
    case TASK_COMPLETE: return "TASK_COMPLETE";
    default: return "UNKNOWN";
  }
}

void Task2_1::lineFollowPID(uint16_t *sensorValues) {
  unsigned long position = qtr->readLineWhite(sensorValues);
  long error = CENTER - (long)position;
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

void Task2_1::handleLeftTurn(uint16_t *sensorValues) {
  // Rotate left until center sensors see white
  while (true) {
    qtr->readLineWhite(sensorValues);
    int wc = 0;
    if (sensorValues[2] < WHITE_THRESHOLD) wc++;
    if (sensorValues[3] < WHITE_THRESHOLD) wc++;
    if (sensorValues[4] < WHITE_THRESHOLD) wc++;
    if (sensorValues[5] < WHITE_THRESHOLD) wc++;
    if (wc >= 2) break;
    motors->turnLeft(120);
    delay(5);
  }
  motors->stop();
}

void Task2_1::handleRightTurn(uint16_t *sensorValues) {
  // Rotate right until center sensors see white
  while (true) {
    qtr->readLineWhite(sensorValues);
    int wc = 0;
    if (sensorValues[2] < WHITE_THRESHOLD) wc++;
    if (sensorValues[3] < WHITE_THRESHOLD) wc++;
    if (sensorValues[4] < WHITE_THRESHOLD) wc++;
    if (sensorValues[5] < WHITE_THRESHOLD) wc++;
    if (wc >= 2) break;
    motors->turnRight(120);
    delay(5);
  }
  motors->stop();
}

void Task2_1::handleIntersection(uint16_t *sensorValues) {
  // Straight section: move forward ~20cm, then recover to line
  if (straightCounter == 0) {
    noInterrupts();
    long startR = rightPulse;
    interrupts();
    // store starting pulse negated in straightCounter (matches original algorithm)
    straightCounter = -startR;
  }

  motors->forward(baseSpeed);
  delay(5);

  // check pulses traveled using right encoder as primary (original behavior)
  noInterrupts();
  long rDelta = abs(rightPulse + straightCounter);
  interrupts();

  if (rDelta >= maxStraightPulses) {
    motors->stop();
    delay(100);
    Serial.println(">> Dash line missed: moving backward until white detected...");

    // Move backward slowly until middle sensors detect white line
    while (true) {
      qtr->readRaw(sensorValues);
      if (sensorValues[3] < WHITE_THRESHOLD || sensorValues[4] < WHITE_THRESHOLD) break;
      motors->backward(80);
      delay(5);
    }
    motors->stop();
    delay(50);

    // Instead of time-based turn, do a precise 5° left turn using encoders
    // turnAngle(angleDeg, wheelDiameterCm, encoderPPR, wheelBaseCm, maxSpeed, minSpeed)
    turnAngle(10.0f, 6.5f, 1650, 14.25f, 120, 30);
    delay(50);

    // Small rotation to finalize alignment (original used setMotor(50,-50) for a short burst)
    // motors->turnRight(50);
    // delay(120);
    // motors->stop();
    // delay(50);

    straightCounter = 0;
  }
}

void Task2_1::rotate180Left() {
  // Rotate left 180°: approximately 1600ms at speed 120
  motors->turnLeft(120);
  delay(1600);
  motors->stop();
  delay(200);
}

void Task2_1::forwardUntilWhiteIR() {
  // Move forward until both IR sensors detect white
  while (true) {
    int leftIR = digitalRead(leftIRPin);
    int rightIR = digitalRead(rightIRPin);

    if (leftIR == LOW && rightIR == LOW) {
      motors->stop();
      delay(200);
      break;
    }

    motors->forward(90);
    delay(5);
  }
}

bool Task2_1::update() {
  if (taskDone) return true;

  uint16_t sensorValues[8];

  // Read sensors
  int leftIR = digitalRead(leftIRPin);
  int rightIR = digitalRead(rightIRPin);
  qtr->readRaw(sensorValues);

  // Count sensor states
  int whiteCount = 0;
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
    if (sensorValues[i] > BLACK_THRESHOLD) blackCount++;
  }

  // State machine
  if (state == LINE_FOLLOW) {
    // Check for exit condition: both IR sensors see white
    if (leftIR == LOW && rightIR == LOW) {
      state = FINAL_STOP_WAIT;
      motors->stop();
      delay(500);
      return false;
    }

    // Check for left edge case
    if (leftIR == LOW && rightIR == HIGH && whiteCount <= 2) {
      handleLeftTurn(sensorValues);
      return false;
    }

    // Check for right edge case
    if (rightIR == LOW && leftIR == HIGH && whiteCount <= 2) {
      handleRightTurn(sensorValues);
      return false;
    }

    // Check for intersection (long black section)
    if (blackCount >= 6) {
      // Let handleIntersection manage encoder snapshot and distance tracking
      handleIntersection(sensorValues);
      return false;
    } else {
      straightCounter = 0;
    }

    // Normal line following
    lineFollowPID(sensorValues);
  }
  else if (state == FINAL_STOP_WAIT) {
    Serial.println(">> Doing 25cm forward...");
    mover->moveForwardCm(25);
    state = MOVE_5CM_BACK;
  }
  else if (state == MOVE_5CM_BACK) {
    Serial.println(">> Doing 5cm backward...");
    mover->moveForwardCm(-5);
    state = ROTATE_180;
  }
  else if (state == ROTATE_180) {
    Serial.println(">> Rotating 180 degrees LEFT...");
    rotate180Left();
    state = FORWARD_UNTIL_WHITE;
  }
  else if (state == FORWARD_UNTIL_WHITE) {
    Serial.println(">> Moving forward until white line detected...");
    forwardUntilWhiteIR();
    Serial.println(">> White line detected. TASK COMPLETE.");
    state = TASK_COMPLETE;
    taskDone = true;
    return true;
  }

  return false;
}
