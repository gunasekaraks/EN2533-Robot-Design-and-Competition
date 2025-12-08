#include "RampTask.h"
#include "LineSensors.h"
#include "MotorControl.h"
#include "MoveController.h"

// Threshold to consider a QTR sensor reading as "white" (on your surface)
static const uint16_t WHITE_THRESHOLD = 300;

RampTask::RampTask(LineSensors* qtr_, MotorControl* motors_, MoveController* mover_, int leftIR, int rightIR)
  : qtr(qtr_), motors(motors_), mover(mover_), leftIRPin(leftIR), rightIRPin(rightIR), started(false) {}

void RampTask::begin() {
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  started = true;
}

bool RampTask::update() {
  if (!started) return false;

  // Read IR sensors
  int leftIR = digitalRead(leftIRPin);
  int rightIR = digitalRead(rightIRPin);

  // Read raw QTR sensor values through the LineSensors wrapper
  uint8_t cnt = qtr->count();
  uint16_t *sensorValues = new uint16_t[cnt];
  qtr->readRaw(sensorValues);

  // Count white detections (IR sensors are active-low on your board)
  int whiteCount = 0;
  if (leftIR == 0) whiteCount++;
  if (rightIR == 0) whiteCount++;
  for (uint8_t i = 0; i < cnt; i++) {
    if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
  }

  delete[] sensorValues;

  // If enough sensors detect white, stop and report completion
  if (whiteCount >= 5) {
    motors->stop();
    Serial.println("Ramp: 5+ SENSORS DETECTED WHITE -> STOP");
    return true; // task complete
  }

  // Otherwise keep moving forward at a steady speed
  motors->forward(200);
  return false; // not complete yet
}
