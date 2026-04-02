#include "Task1.h"
#include "LineSensors.h"
#include "MotorControl.h"

// External encoder variables
extern volatile long rightPulse;
extern volatile long leftPulse;

Task1::Task1(LineSensors* sensors, MotorControl* motors,
             float wheelDiameter, int encoderPPR, float wheelBase)
  : sensors(sensors), motors(motors),
    wheelDiameter(wheelDiameter), encoderPPR(encoderPPR), wheelBase(wheelBase) {
  // Initialize grid map
  for (int r = 1; r <= 9; r++) {
    for (int c = 1; c <= 9; c++) {
        
      gridMap[r][c] = 0;
      knownMap[r][c] = false;
    }
  }
   
}

void Task1::begin() {
  // Initialize pins
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  pinMode(redPin, OUTPUT); pinMode(greenPin, OUTPUT); pinMode(bluePin, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Set color sensor to 20% frequency
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Reset state
  curRow = 1;
  curCol = 0;
  currentDirection = EAST;
  junctionDetected = false;
  inDeliveryMode = false;
  inAvoidanceMode = false;
  lastError = 0;
  integral = 0;

  // Reset encoders
  noInterrupts();
  rightPulse = 0;
  leftPulse = 0;
  interrupts();

  taskStarted = true;
  taskComplete = false;

  Serial.println("[Task1] Grid solver starting. Moving to first junction (1,1)...");
  lineFollowNormal();  // Move to first junction
  gridMap[curRow][curCol] = 0;  // Mark starting junction as empty
}

void Task1::setMotor(int left, int right) {
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

void Task1::stopMotors() {
  motors->stop();
}

void Task1::setColor(int r, int g, int b) {
  // Defensive: ensure pins are outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Debug print the PWM values being written
  Serial.print("[Task1] setColor -> R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);

  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluePin, b);
}

int Task1::readColorAverage(bool s2State, bool s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(50);
  int frequency = pulseIn(SENSOR_OUT, LOW);
  if (frequency == 0) frequency = 30000;
  return frequency;
}

String Task1::detectColorFromFreq(int r, int g, int b) {
  if (r < 240 && g < 240 && b < 240) return "WHITE";
  if (r < 280 && r < g - 50 && r < b - 40) return "RED";
  if (b < 260 && b < r - 50 && b < g - 80) return "BLUE";
  if (g < r - 40 && g < b - 20) return "GREEN";
  return "UNKNOWN";
}

int Task1::detectNextBoxColor() {
  int r = readColorAverage(LOW, LOW);
  int g = readColorAverage(HIGH, HIGH);
  int b = readColorAverage(LOW, HIGH);

  // Print raw sensor frequencies for debugging
  Serial.print("[Task1] Color raw -> R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.println(b);

  String col = detectColorFromFreq(r, g, b);
  Serial.print("[Task1] detectNextBoxColor -> "); Serial.println(col);

  if (col == "RED") {
    setColor(255, 0, 0);
    lastDetectedBoxColor = 2;
    return 2;
  } else if (col == "GREEN") {
    setColor(0, 255, 0);
    lastDetectedBoxColor = 3;
    return 3;
  } else if (col == "BLUE") {
    setColor(0, 0, 255);
    lastDetectedBoxColor = 4;
    return 4;
  } else {
    setColor(0, 0, 0);
    lastDetectedBoxColor = 0;
    return 0;
  }
}

int Task1::detectNextBoxColorMajority(int samples) {
  int countsRed = 0, countsGreen = 0, countsBlue = 0, countsUnknown = 0;
  Serial.print("[Task1] Majority sampling (n="); Serial.print(samples); Serial.println(")");
  for (int i = 0; i < samples; i++) {
    int r = readColorAverage(LOW, LOW);
    int g = readColorAverage(HIGH, HIGH);
    int b = readColorAverage(LOW, HIGH);
    String col = detectColorFromFreq(r, g, b);

    Serial.print("  sample "); Serial.print(i+1);
    Serial.print(" -> R="); Serial.print(r);
    Serial.print(" G="); Serial.print(g);
    Serial.print(" B="); Serial.print(b);
    Serial.print(" => "); Serial.println(col);

    if (col == "RED") countsRed++;
    else if (col == "GREEN") countsGreen++;
    else if (col == "BLUE") countsBlue++;
    else countsUnknown++;

    delay(25);
  }

  Serial.print("[Task1] counts: R="); Serial.print(countsRed);
  Serial.print(" G="); Serial.print(countsGreen);
  Serial.print(" B="); Serial.print(countsBlue);
  Serial.print(" U="); Serial.println(countsUnknown);

  int colorCode = 0;
  if (countsRed >= 5) colorCode = 2;
  else if (countsGreen >= 5) colorCode = 3;
  else if (countsBlue >= 5) colorCode = 4;

  if (colorCode == 2) setColor(255, 0, 0);
  else if (colorCode == 3) setColor(0, 255, 0);
  else if (colorCode == 4) setColor(0, 0, 255);
  else setColor(0, 0, 0);

  Serial.print("[Task1] Majority result code="); Serial.println(colorCode);
  lastDetectedBoxColor = colorCode;
  return colorCode;
}

bool Task1::isNextObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return false;
  
  int distance = duration * 0.0343 / 2;
  return (distance <= 13);
}

const char* Task1::directionToString(Direction dir) {
  switch (dir) {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case SOUTH: return "SOUTH";
    case WEST: return "WEST";
  }
  return "UNKNOWN";
}

void Task1::moveForward(int acm) {
  int maxSpeed = 150;
  int minSpeed = 80;
  
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();
  
  float wheelCirc = 3.1416 * wheelDiameter;
  long targetPulses = (long)((acm / wheelCirc) * encoderPPR);
  
  float Kp = 0.45, Ki = 0.0, Kd = 0.09;
  float integral = 0, lastError = 0;
  unsigned long lastTime = millis();
  
  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();
    
    long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;
    float error = (float)(targetPulses - avgPulses);
    if (error <= 0) break;
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    if (dt > 0.2) dt = 0.2;
    
    integral += error * dt;
    if (integral > maxSpeed * 2) integral = maxSpeed * 2;
    if (integral < -maxSpeed * 2) integral = -maxSpeed * 2;
    
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    int forwardSpeed = (int)output;
    if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
    if (forwardSpeed < 0) forwardSpeed = 0;
    if (forwardSpeed < minSpeed && error > 20) forwardSpeed = minSpeed;
    
    setMotor(forwardSpeed, forwardSpeed);
    delay(5);
  }
  
  stopMotors();
}

void Task1::moveBackward(int acm) {
  int maxSpeed = 150;
  int minSpeed = 80;
  
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();
  
  float wheelCirc = 3.1416 * wheelDiameter;
  long targetPulses = (long)((acm / wheelCirc) * encoderPPR);
  
  float Kp = 0.45, Ki = 0.0, Kd = 0.0;
  float integral = 0, lastError = 0;
  unsigned long lastTime = millis();
  
  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();
    
    long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;
    float error = (float)(targetPulses - avgPulses);
    if (error <= 0) break;
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    if (dt > 0.2) dt = 0.2;
    
    integral += error * dt;
    if (integral > maxSpeed * 2) integral = maxSpeed * 2;
    if (integral < -maxSpeed * 2) integral = -maxSpeed * 2;
    
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    int backSpeed = (int)output;
    if (backSpeed > maxSpeed) backSpeed = maxSpeed;
    if (backSpeed < 0) backSpeed = 0;
    if (backSpeed < minSpeed && error > 20) backSpeed = minSpeed;
    
    setMotor(-backSpeed, -backSpeed);
    delay(5);
  }
  
  stopMotors();
}

void Task1::updatePositionAfterForward() {
  switch (currentDirection) {
    case NORTH:
      curRow++;
      if (curRow > 9) curRow = 9;
      break;
    case EAST:
      curCol++;
      if (curCol > 9) curCol = 9;
      break;
    case SOUTH:
      curRow--;
      if (curRow < 1) curRow = 1;
      break;
    case WEST:
      curCol--;
      if (curCol < 1) curCol = 1;
      break;
  }
  
  if (currentDirection == EAST) junctionCount = curCol;
  else if (currentDirection == WEST) junctionCount = 10 - curCol + 1;
  else junctionCount++;
}

void Task1::hardLeftTurn90() {
  noInterrupts();
  moveForward(2);
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();
  
  int maxSpeed = 150, minSpeed = 80;
  float wheelCircumference = 3.1416 * wheelDiameter;
  float arcLength = 3.1416 * wheelBase / 4.0;
  long targetPulsesOne = (long)((arcLength / wheelCircumference) * encoderPPR);
  long targetTotal = 2 * targetPulsesOne;
  
  float Kp = 0.4, Ki = 0.0, Kd = 0.0;
  float integral = 0.0, lastError = 0.0;
  unsigned long lastTime = millis();
  
  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();
    
    long rCount = abs(rDelta);
    long lCount = abs(lDelta);
    long currentTotal = rCount + lCount;
    float error = (float)(targetTotal - currentTotal);
    if (error <= 0) break;
    
    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0;
    lastTime = now;
    
    if (dt > 0.0) {
      if (dt > 0.2) dt = 0.2;
      integral += error * dt;
      float maxIntegral = (float)maxSpeed * 2.0;
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;
    }
    
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    int turnSpeed = (int)output;
    if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
    if (turnSpeed < 0) turnSpeed = 0;
    if (turnSpeed < minSpeed && error > 10) turnSpeed = minSpeed;
    
    // Left forward, Right backward
    motors->setLeftMotor(turnSpeed, true);
    motors->setRightMotor(turnSpeed, false);
    
    delay(5);
  }
  
  stopMotors();
  
  switch (currentDirection) {
    case NORTH: currentDirection = WEST; break;
    case EAST: currentDirection = NORTH; break;
    case SOUTH: currentDirection = EAST; break;
    case WEST: currentDirection = SOUTH; break;
  }
  
  Serial.print("Turned LEFT to face ");
  Serial.println(directionToString(currentDirection));
}

void Task1::hardRightTurn90() {
  noInterrupts();
  moveForward(2);
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();
  
  int maxSpeed = 150, minSpeed = 80;
  float wheelCircumference = 3.1416 * wheelDiameter;
  float arcLength = 3.1416 * wheelBase / 4.0;
  long targetPulsesOne = (long)((arcLength / wheelCircumference) * encoderPPR);
  long targetTotal = 2 * targetPulsesOne;
  
  float Kp = 0.4, Ki = 0.0, Kd = 0.0;
  float integral = 0.0, lastError = 0.0;
  unsigned long lastTime = millis();
  
  while (true) {
    noInterrupts();
    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;
    interrupts();
    
    long rCount = abs(rDelta);
    long lCount = abs(lDelta);
    long currentTotal = rCount + lCount;
    float error = (float)(targetTotal - currentTotal);
    if (error <= 0) break;
    
    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0;
    lastTime = now;
    
    if (dt > 0.0) {
      if (dt > 0.2) dt = 0.2;
      integral += error * dt;
      float maxIntegral = (float)maxSpeed * 2.0;
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;
    }
    
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    int turnSpeed = (int)output;
    if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
    if (turnSpeed < 0) turnSpeed = 0;
    if (turnSpeed < minSpeed && error > 10) turnSpeed = minSpeed;
    
    // Left backward, Right forward
    motors->setLeftMotor(turnSpeed, false);
    motors->setRightMotor(turnSpeed, true);
    
    delay(5);
  }
  
  stopMotors();
  
  switch (currentDirection) {
    case NORTH: currentDirection = EAST; break;
    case EAST: currentDirection = SOUTH; break;
    case SOUTH: currentDirection = WEST; break;
    case WEST: currentDirection = NORTH; break;
  }
  
  Serial.print("Turned RIGHT to face ");
  Serial.println(directionToString(currentDirection));
}

void Task1::turnAround180() {
  hardLeftTurn90();
  delay(120);
  hardLeftTurn90();
}

void Task1::lineFollowNormal() {
  float Kp = 0.04;
  float Ki = 0.00;
  float Kd = 0.04;
  lastError = 0;
  integral = 0.0;
  
  while (true) {
    uint16_t sensorValues[8];
    sensors->readRaw(sensorValues);
    
    unsigned long position = sensors->readLineWhite(sensorValues);
    
    int whiteCount = 0;
    for (int i = 0; i < 8; i++) {
      if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
    }
    
    // Junction detection
    if (!junctionDetected) {
      if (whiteCount >= JUNCTION_WHITE_COUNT) {
        junctionDetected = true;
        stopMotors();
        if (autoUpdatePosition) {
          updatePositionAfterForward();
        }
        Serial.print("Junction detected at R");
        Serial.print(curRow);
        Serial.print(" C");
        Serial.print(curCol);
        Serial.print(" - Moving ");
        Serial.println(directionToString(currentDirection));
        return;
      }
    }
    
    // Reset detection
    if (junctionDetected) {
      if (whiteCount < 6) {
        junctionDetected = false;
      }
    }
    
    long error = CENTER - position;
    integral += error;
    long derivative = error - lastError;
    lastError = error;
    
    float correction = Kp * error + Ki * integral + Kd * derivative;
    
    int leftSpeed = constrain((int)(baseSpeed + correction), 0, 255);
    int rightSpeed = constrain((int)(baseSpeed - correction), 0, 255);
    
    setMotor(leftSpeed, rightSpeed);
    
    delay(10);
  }
}

void Task1::goDownOneRowFromEnd() {
  if (curRow >= 9) return;
  
  Serial.println("Going down to next row...");
  Direction originalHorizontalDir = currentDirection;
  
  if (currentDirection == EAST) {
    hardLeftTurn90();
  } else {
    hardRightTurn90();
  }
  
  lineFollowNormal();
  
  if (originalHorizontalDir == EAST) {
    hardLeftTurn90();
  } else {
    hardRightTurn90();
  }
  
  Serial.print("Moved down to row ");
  Serial.print(curRow);
  Serial.print(" col ");
  Serial.print(curCol);
  Serial.print(" facing ");
  Serial.println(directionToString(currentDirection));
}

int Task1::detectBoxAtCurrentPosition() {
  int color = detectNextBoxColor();
  return color;
}

int Task1::colorsensor() {
  // Central wrapper for color sensing: perform multiple samples and return majority result
  int samples = 10;
  int result = detectNextBoxColorMajority(samples);
  Serial.print("[Task1] colorsensor() -> "); Serial.println(result);
  return result;
}

void Task1::pickUpBox() {
  Serial.println("Picking up box...");
  delay(500);
}

void Task1::placeBox() {
  Serial.println("Placing box...");
  delay(500);
}

bool Task1::findPathBFS(int startRow, int startCol, int targetRow, int targetCol,
                        int path[81][2], int& pathLen) {
  int visited[10][10] = {0};
  struct Node { int row, col, distance; };
  Node queue[81];
  int parent[10][10][2];
  
  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      parent[r][c][0] = -1;
      parent[r][c][1] = -1;
    }
  }
  
  int queueFront = 0, queueRear = 0;
  queue[queueRear].row = startRow;
  queue[queueRear].col = startCol;
  queue[queueRear].distance = 0;
  queueRear++;
  visited[startRow][startCol] = 1;
  
  while (queueFront < queueRear) {
    Node current = queue[queueFront++];
    
    if (current.row == targetRow && current.col == targetCol) {
      pathLen = 0;
      int r = targetRow, c = targetCol;
      
      while (r != -1 && c != -1) {
        path[pathLen][0] = r;
        path[pathLen][1] = c;
        pathLen++;
        
        int pr = parent[r][c][0];
        int pc = parent[r][c][1];
        r = pr;
        c = pc;
      }
      
      for (int i = 0; i < pathLen / 2; i++) {
        int tempR = path[i][0];
        int tempC = path[i][1];
        path[i][0] = path[pathLen - 1 - i][0];
        path[i][1] = path[pathLen - 1 - i][1];
        path[pathLen - 1 - i][0] = tempR;
        path[pathLen - 1 - i][1] = tempC;
      }
      
      return true;
    }
    
    int dr[] = {1, 0, -1, 0};
    int dc[] = {0, 1, 0, -1};
    
    for (int dir = 0; dir < 4; dir++) {
      int nr = current.row + dr[dir];
      int nc = current.col + dc[dir];
      
      if (nr >= 1 && nr <= 9 && nc >= 1 && nc <= 9 && !visited[nr][nc]) {
        bool canMove = true;
        if (knownMap[nr][nc]) {
          if (gridMap[nr][nc] == 1) canMove = false;
        }
        
        if (canMove) {
          visited[nr][nc] = 1;
          parent[nr][nc][0] = current.row;
          parent[nr][nc][1] = current.col;
          
          queue[queueRear].row = nr;
          queue[queueRear].col = nc;
          queue[queueRear].distance = current.distance + 1;
          queueRear++;
        }
      }
    }
  }
  
  return false;
}

bool Task1::navigateToJunction(int startRow, int startCol, Direction startDir,
                               int targetRow, int targetCol) {
  Serial.print("Navigation: From R");
  Serial.print(startRow);
  Serial.print(" C");
  Serial.print(startCol);
  Serial.print(" to R");
  Serial.print(targetRow);
  Serial.print(" C");
  Serial.println(targetCol);
  
  int path[81][2];
  int pathLen = 0;
  
  if (!findPathBFS(startRow, startCol, targetRow, targetCol, path, pathLen)) {
    Serial.println("ERROR: No path found to target!");
    return false;
  }
  
  Serial.print("Path found with ");
  Serial.print(pathLen);
  Serial.println(" junctions");
  
  curRow = startRow;
  curCol = startCol;
  currentDirection = startDir;
  
  for (int i = 1; i < pathLen; i++) {
    int nextRow = path[i][0];
    int nextCol = path[i][1];
    
    Direction neededDir;
    if (nextRow > curRow) neededDir = NORTH;
    else if (nextRow < curRow) neededDir = SOUTH;
    else if (nextCol > curCol) neededDir = EAST;
    else neededDir = WEST;
    
    while (currentDirection != neededDir) {
      if ((currentDirection == NORTH && neededDir == EAST) ||
          (currentDirection == EAST && neededDir == SOUTH) ||
          (currentDirection == SOUTH && neededDir == WEST) ||
          (currentDirection == WEST && neededDir == NORTH)) {
        hardRightTurn90();
      } else {
        hardLeftTurn90();
      }
    }
    
    int probeRow = curRow, probeCol = curCol;
    switch (currentDirection) {
      case EAST: probeCol++; break;
      case WEST: probeCol--; break;
      case NORTH: probeRow++; break;
      case SOUTH: probeRow--; break;
    }
    
    if (!(probeRow < 1 || probeRow > 9 || probeCol < 1 || probeCol > 9)) {
      if (!knownMap[probeRow][probeCol]) {
        bool obs = isNextObstacle();
        knownMap[probeRow][probeCol] = true;
        gridMap[probeRow][probeCol] = obs ? 1 : 0;
        if (obs) return false;
      } else if (gridMap[probeRow][probeCol] == 1) {
        return false;
      }
    }
    
    lineFollowNormal();
  }
  
  if (curRow == targetRow && curCol == targetCol) {
    int boxColor = detectBoxAtCurrentPosition();
    if (boxColor != 0 && !inDeliveryMode) {
      Serial.print("Box detected at target! Color: ");
      Serial.println(boxColor);
      pickUpBox();
      deliverBoxAndReturn(boxColor, curRow, curCol, currentDirection);
    }
  }
  
  return true;
}

void Task1::obstacleAvoid() {
  Serial.println("Avoiding obstacle (BFS-assisted)...");
  Direction originalDirection = currentDirection;
  
  if (!inAvoidanceMode) {
    inAvoidanceMode = true;
    originalAvoidanceDirection = originalDirection;
  }
  
  // Back up to previous junction
  moveBackward(2);
  
  uint16_t sensorValues[8];
  sensors->readRaw(sensorValues);
  
  int whiteCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
  }
  
  if (whiteCount >= JUNCTION_WHITE_COUNT) {
    Serial.print("Backed up to R");
    Serial.print(curRow);
    Serial.print(" C");
    Serial.println(curCol);
  }
  
  // Mark obstacle ahead
  int obsRow = curRow, obsCol = curCol;
  switch (currentDirection) {
    case EAST: obsCol++; break;
    case WEST: obsCol--; break;
    case NORTH: obsRow++; break;
    case SOUTH: obsRow--; break;
  }
  
  if (!(obsRow < 1 || obsRow > 9 || obsCol < 1 || obsCol > 9)) {
    gridMap[obsRow][obsCol] = 1;
    knownMap[obsRow][obsCol] = true;
  }
  
  inAvoidanceMode = false;
  originalAvoidanceTargetRow = -1;
  originalAvoidanceTargetCol = -1;
}

void Task1::deliverBoxAndReturn(int color, int savedRow, int savedCol, Direction savedDir) {
  Serial.print("Delivering box (color=");
  Serial.print(color);
  Serial.println(")");
  
  int dropRow = 9, dropCol = -1;
  if (color == 4) dropCol = BLUE_DROP_COL;
  else if (color == 3) dropCol = GREEN_DROP_COL;
  else if (color == 2) dropCol = RED_DROP_COL;
  else return;
  
  pickUpBox();
  delay(200);
  
  inDeliveryMode = true;
  
  if (navigateToJunction(curRow, curCol, currentDirection, dropRow, dropCol)) {
    while (currentDirection != EAST) {
      if ((currentDirection == NORTH && EAST == EAST) ||
          (currentDirection == EAST && EAST == SOUTH) ||
          (currentDirection == SOUTH && EAST == WEST) ||
          (currentDirection == WEST && EAST == NORTH)) {
        hardRightTurn90();
      } else {
        hardLeftTurn90();
      }
      delay(100);
    }
    
    placeBox();
    delay(200);
    
    navigateToJunction(curRow, curCol, currentDirection, savedRow, savedCol);
    
    while (currentDirection != savedDir) {
      if ((currentDirection == NORTH && savedDir == EAST) ||
          (currentDirection == EAST && savedDir == SOUTH) ||
          (currentDirection == SOUTH && savedDir == WEST) ||
          (currentDirection == WEST && savedDir == NORTH)) {
        hardRightTurn90();
      } else {
        hardLeftTurn90();
      }
      delay(100);
    }
  }
  
  inDeliveryMode = false;
}

bool Task1::update() {
  if (taskComplete) {
    return true;
  }
  
  // Main grid scanning loop
  if (!taskStarted) {
    return false;
  }
  
  // Check if at end of row
  bool atRowEnd = (currentDirection == EAST && curCol == 9) ||
                  (currentDirection == WEST && curCol == 1);
  
  if (atRowEnd) {
    if (curRow < 9) {
      goDownOneRowFromEnd();
      gridMap[curRow][curCol] = 0;
    } else {
      Serial.println("[Task1] Finished scanning all rows! Task complete.");
      stopMotors();
      taskComplete = true;
      return true;
    }
    return false;
  }
  
  // Probe next junction
  bool nextHasObstacle = isNextObstacle();
  
  int probeRow = curRow, probeCol = curCol;
  switch (currentDirection) {
    case EAST: probeCol++; break;
    case WEST: probeCol--; break;
    case NORTH: probeRow++; break;
    case SOUTH: probeRow--; break;
  }
  
  if (!(probeRow < 1 || probeRow > 9 || probeCol < 1 || probeCol > 9)) {
    if (nextHasObstacle) {
      gridMap[probeRow][probeCol] = 1;
      knownMap[probeRow][probeCol] = true;
    } else {
      knownMap[probeRow][probeCol] = true;
      gridMap[probeRow][probeCol] = 0;
    }
  }
  
  if (!nextHasObstacle) {
    lineFollowNormal();
    gridMap[curRow][curCol] = 0;
  } else {
    Serial.println("Obstacle detected ahead. Moving forward 4cm to inspect...");
    moveForward(4);
    
      // Use centralized colorsensor wrapper for detection
      int color = colorsensor();
    
    if (color == 2 || color == 3 || color == 4) {
      Serial.print("Box detected! Color: ");
      Serial.println(color);
      pickUpBox();
      deliverBoxAndReturn(color, curRow, curCol, currentDirection);
    } else {
      Serial.println("Not a box. Marking as obstacle and executing avoidance.");
      
      int obsRow = curRow, obsCol = curCol;
      switch (currentDirection) {
        case EAST: obsCol++; break;
        case WEST: obsCol--; break;
        case NORTH: obsRow++; break;
        case SOUTH: obsRow--; break;
      }
      
      if (!(obsRow < 1 || obsRow > 9 || obsCol < 1 || obsCol > 9)) {
        gridMap[obsRow][obsCol] = 1;
        knownMap[obsRow][obsCol] = true;
      }
      
      obstacleAvoid();
      gridMap[curRow][curCol] = 0;
    }
  }
  
  delay(60);
  return false;
}
