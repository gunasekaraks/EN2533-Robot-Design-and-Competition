#include "Task5_Arrow.h"

// External encoder variables (declared in main.cpp)
extern volatile long rightPulse;
extern volatile long leftPulse;

Task5_Arrow::Task5_Arrow(LineSensors* sensors, MotorControl* motors, 
                         int leftIRPin, int rightIRPin,
                         float wheelDiameter, int encoderPPR)
  : sensors(sensors), motors(motors), 
    leftIRPin(leftIRPin), rightIRPin(rightIRPin),
    wheelDiameter(wheelDiameter), encoderPPR(encoderPPR) {
  
  // Calculate max straight pulses (40 cm)
  float wheelCirc = 3.1416 * wheelDiameter;
  maxStraightPulses = (long)((40.0 / wheelCirc) * encoderPPR);
}

void Task5_Arrow::begin() {
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  
  finalStop = false;
  waitMiddleAfterRotation = false;
  searchingForLine = true;
  straightCounter = 0;
  lastError = 0;
  integral = 0;
  
  noInterrupts();
  rightPulse = 0;
  leftPulse = 0;
  interrupts();
  
  searchStartTime = millis();
  Serial.println("Task5_Arrow started: Moving forward with encoders until white line detected");
}

void Task5_Arrow::setMotor(int left, int right) {
  // Positive = forward, negative = backward
  if (left >= 0) {
    motors->setLeftMotor(left, true);
  } else {
    motors->setLeftMotor(-left, false);
  }
  
  if (right >= 0) {
    motors->setRightMotor(right, true);
  } else {
    motors->setRightMotor(-right, false);
  }
}

void Task5_Arrow::stopMotors() {
  motors->stop();
}

void Task5_Arrow::rotateLeft90() {
  setMotor(160, -160);
  delay(300);  
  setMotor(0, 0);
}

void Task5_Arrow::rotateLeft() {
  setMotor(120, -120);
}

void Task5_Arrow::rotateRight() {
  setMotor(-120, 120);
}

void Task5_Arrow::goStraight() {
  setMotor(baseSpeed, baseSpeed);
}

void Task5_Arrow::moveDistance(float dist_cm) {
  noInterrupts();
  long startR = rightPulse;
  long startL = leftPulse;
  interrupts();

  float wheelCirc = 3.1416 * wheelDiameter;
  long targetPulses = abs((dist_cm / wheelCirc) * encoderPPR);

  float Kp_move = 0.45f;
  float Ki_move = 0.0f;
  float Kd_move = 0.0f;

  float integral_move = 0;
  float lastErr = 0;
  unsigned long lastT = millis();

  while (true) {
    noInterrupts();
    long rDelta = abs(rightPulse - startR);
    long lDelta = abs(leftPulse - startL);
    interrupts();

    long avg = (rDelta + lDelta) / 2;
    float error = (float)(targetPulses - avg);
    if (error <= 0) break;

    unsigned long now = millis();
    float dt = (now - lastT) / 1000.0f;
    if (dt > 0.2f) dt = 0.2f;
    lastT = now;

    integral_move += error * dt;
    float derivative = (error - lastErr) / dt;
    lastErr = error;

    float output = Kp_move * error + Ki_move * integral_move + Kd_move * derivative;
    int speed = (int)output;
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < minSpeed && error > 30) speed = minSpeed;
    if (speed < 0) speed = 0;

    if (dist_cm > 0) setMotor(speed, speed);
    else setMotor(-speed, -speed);

    delay(5);
  }

  stopMotors();
  delay(200);
}

void Task5_Arrow::moveForwardUntilWhite() {
  // Move forward with encoder-based speed control until white line detected
  uint16_t sensorValues[8];
  
  noInterrupts();
  long startR = rightPulse;
  long startL = leftPulse;
  interrupts();
  
  float wheelCirc = 3.1416 * wheelDiameter;
  
  // Use encoder PID to keep wheels balanced
  float Kp_balance = 0.5f;
  float lastErr = 0;
  
  while (searchingForLine) {
    // Read sensors
    sensors->readRaw(sensorValues);
    
    // Check for white line (any middle sensor detects white)
    if (sensorValues[2] < WHITE_THRESHOLD || 
        sensorValues[3] < WHITE_THRESHOLD ||
        sensorValues[4] < WHITE_THRESHOLD ||
        sensorValues[5] < WHITE_THRESHOLD) {
      Serial.println("White line detected! Stopping search.");
      stopMotors();
      searchingForLine = false;
      delay(200);
      return;
    }
    
    // Encoder balance control
    noInterrupts();
    long rDelta = rightPulse - startR;
    long lDelta = leftPulse - startL;
    interrupts();
    
    long error = rDelta - lDelta;  // Positive = right faster
    float correction = Kp_balance * error;
    
    int leftSpeed = 100 - (int)correction;
    int rightSpeed = 100 + (int)correction;
    
    // Constrain speeds
    if (leftSpeed > 150) leftSpeed = 150;
    if (leftSpeed < 30) leftSpeed = 30;
    if (rightSpeed > 150) rightSpeed = 150;
    if (rightSpeed < 30) rightSpeed = 30;
    
    setMotor(leftSpeed, rightSpeed);
    
    // Safety timeout: if no white line found after 3 seconds, stop
    if (millis() - searchStartTime > 3000) {
      Serial.println("Timeout: No white line found. Stopping.");
      stopMotors();
      searchingForLine = false;
      return;
    }
    
    delay(5);
  }
}

bool Task5_Arrow::update() {
  // Initial phase: move forward with encoders until white line detected
  if (searchingForLine) {
    moveForwardUntilWhite();
    return false;  // Keep running until white line found
  }
  
  // Task complete check
  if (finalStop == true) {
    stopMotors();
    Serial.println("Task5_Arrow COMPLETE!");
    return true;
  }

  int leftIR = digitalRead(leftIRPin);
  int rightIR = digitalRead(rightIRPin);

  uint16_t sensorValues[8];
  sensors->readRaw(sensorValues);

  int blackCount = 0, whiteCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) blackCount++;
    if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
  }

  // After rotation, wait for middle sensor to detect white
  if (waitMiddleAfterRotation) {
    int mid = sensorValues[3];
    if (mid < WHITE_THRESHOLD) {
      waitMiddleAfterRotation = false;
      Serial.println("Middle sensor white detected → resume PID line following");
      setMotor(0, 0);
      delay(500);
    } else {
      setMotor(0, 0);
      delay(500);
    }
    return false;
  }

  // Both IR sensors LOW → task complete (both sensors see white)
  if (leftIR == LOW && rightIR == LOW) {
    finalStop = true;
    return false;  // Will return true on next update call
  }

  // Right IR detects white (left sees black)
  if (leftIR == LOW && rightIR == HIGH) {
    rotateRight();
    delay(80);
    return false;
  }

  // Left IR detects white (right sees black) → rotate left 90°
  if (leftIR == HIGH && rightIR == LOW && !waitMiddleAfterRotation) {
    setMotor(0, 0);
    delay(500);
    rotateLeft90();
    waitMiddleAfterRotation = true;
    return false;
  }

  // Dash-line recovery: if too many black sensors detected
  if (blackCount >= 6) {
    if (straightCounter == 0) {
      noInterrupts();
      long startR = rightPulse;
      interrupts();
      straightCounter = -startR;
    }

    goStraight();
    delay(5);

    noInterrupts();
    long rDelta = abs(rightPulse + straightCounter);
    interrupts();

    if (rDelta >= maxStraightPulses) {
      stopMotors();
      delay(100);
      Serial.println(">> Dash line missed: moving backward until white detected...");

      while (true) {
        sensors->readRaw(sensorValues);
        if (sensorValues[2] < WHITE_THRESHOLD || 
            sensorValues[3] < WHITE_THRESHOLD ||
            sensorValues[4] < WHITE_THRESHOLD ||
            sensorValues[5] < WHITE_THRESHOLD) break;

        setMotor(-80, -80);
        delay(5);
      }

      stopMotors();
      delay(50);

      Serial.println(">> Slight right rotation 5° before dash line...");
      setMotor(100, -100);
      delay(250);
      stopMotors();
      delay(50);

      straightCounter = 0;
    }

    return false;
  } 
  else {
    straightCounter = 0;
  }

  // Normal PID line following
  unsigned long position = sensors->readLineWhite(sensorValues);
  long error = CENTER - position;

  integral += error;
  long derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotor(constrain(leftSpeed, 0, 255),
           constrain(rightSpeed, 0, 255));

  delay(5);
  return false;
}
