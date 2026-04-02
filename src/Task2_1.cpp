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
  moveTargetPulses = 0;
  moveIntegral = 0.0f;
  moveLastError = 0.0f;
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
    turnAngle(-5.0f, 6.5f, 1650, 14.25f, 120, 30);
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

void Task2_1::startMovement(int distanceCm, bool forward) {
  // Initialize movement state for non-blocking encoder-based move
  noInterrupts();
  moveStartRight = rightPulse;
  moveStartLeft = leftPulse;
  interrupts();

  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * 6.5f;  // wheelDiameter = 6.5 cm (hardcoded to match MoveController)
  moveTargetPulses = (long)(((float)distanceCm / wheelCirc) * 1650.0f);  // encoderPPR = 1650

  moveLastTime = millis();
  moveStartTime = moveLastTime;
  // Timeout: proportional to distance (ms per cm), with a sensible floor
  unsigned long perCmMs = 250; // 250 ms per cm
  moveTimeoutMs = (unsigned long)(abs(distanceCm) * perCmMs);
  if (moveTimeoutMs < 2000) moveTimeoutMs = 2000;
  moveIntegral = 0.0f;
  moveLastError = 0.0f;

  Serial.print(">> Starting ");
  if (forward) Serial.print("forward");
  else Serial.print("backward");
  Serial.print(" movement: "); Serial.print(distanceCm); Serial.println(" cm");
}

bool Task2_1::updateMovement(bool forward) {
  // Non-blocking movement update. Returns true when movement is complete.
  noInterrupts();
  long rDelta = rightPulse - moveStartRight;
  long lDelta = leftPulse - moveStartLeft;
  interrupts();

  long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;
  float error = (float)(moveTargetPulses - avgPulses);

  if (error <= 0.0f) {
    // Movement complete
    motors->stop();
    Serial.println(">> Movement complete");
    return true;
  }

  // Check for timeout
  if ((unsigned long)(millis() - moveStartTime) > moveTimeoutMs) {
    motors->stop();
    Serial.println(">> Movement TIMEOUT - aborting move");
    return true; // treat as complete so state machine can continue
  }

  unsigned long now = millis();
  float dt = ((float)(now - moveLastTime)) / 1000.0f;
  moveLastTime = now;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.2f) dt = 0.2f;

  moveIntegral += error * dt;
  float maxIntegral = 150.0f * 2.0f;  // MAX_MOVE_SPEED = 150
  if (moveIntegral > maxIntegral) moveIntegral = maxIntegral;
  if (moveIntegral < -maxIntegral) moveIntegral = -maxIntegral;

  float derivative = (error - moveLastError) / dt;
  moveLastError = error;

  // Use same PID as MoveController: Kp=0.45, Ki=0.0, Kd=0.09
  float output = 0.45f * error + 0.0f * moveIntegral + 0.09f * derivative;
  int speed = (int)output;
  if (speed > 150) speed = 150;  // MAX_MOVE_SPEED
  if (speed < 0) speed = 0;
  if (speed < 50 && error > 20.0f) speed = 50;  // MIN_MOVE_SPEED

  // Drive motors using MotorControl interface
  if (forward) {
    motors->forward(speed);
  } else {
    motors->backward(speed);
  }

  return false;  // Movement not yet complete
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
    Serial.println(">> Starting 30cm forward movement...");
    startMovement(30, true);
    state = MOVE_25CM;
  }
  else if (state == MOVE_25CM) {
    if (updateMovement(true)) {
      // Movement complete, start 5cm backward
      Serial.println(">> Starting 5cm backward movement...");
      startMovement(5, false);
      state = MOVE_5CM_BACK;
    } else {
      // If the forward move appears stuck, enforce timeout here and proceed
      if ((unsigned long)(millis() - moveStartTime) > moveTimeoutMs) {
        Serial.println(">> MOVE_25CM timeout - aborting forward move and proceeding");
        motors->stop();
        // Start backward movement to continue the sequence
        startMovement(5, false);
        state = MOVE_5CM_BACK;
      }
    }
  }
  else if (state == MOVE_5CM_BACK) {
    if (updateMovement(false)) {
      // Backward movement complete, start 180° rotation
      Serial.println(">> Rotating 180 degrees LEFT...");
      rotate180Left();
      state = ROTATE_180;
    }
  }
  else if (state == ROTATE_180) {
    // Rotation is blocking in rotate180Left(), so immediately move to next state
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
