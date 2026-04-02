// Task1_Monolithic.cpp
// Monolithic Task1 grid solver code (user-provided sketch)
// Called from main.cpp's runTask1() function

#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>

// -------------------- QTR / Sensors --------------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
static const uint8_t sensorPins[SensorCount] = {52, 50, 48, 46, 44, 42, 40, 38};

// -------------------- Motor pins --------------------
const int ENA = 10, IN1 = 22, IN2 = 24;  // Left motor
const int ENB = 11, IN3 = 26, IN4 = 28;  // Right motor

// -------------------- PID --------------------
int baseSpeed = 140;
int junctionCount = 0;

// Calibration variables
int Rmin = 99999, Rmax = 0;
int Gmin = 99999, Gmax = 0;
int Bmin = 99999, Bmax = 0;

// Thresholds
int redThresh, greenThresh, blueThresh;

// -------------------- Grid tracking --------------------
int curRow = 1;
int curCol = 0;      // Start BEFORE first junction
bool autoUpdatePosition = true;

// New Direction tracking
enum Direction { NORTH, EAST, SOUTH, WEST };
Direction currentDirection = EAST; // Initial direction is EAST (moving right)

// Map: 0 empty, 1 obstacle, 2 RED, 3 GREEN, 4 BLUE
// Row 1 is SOUTH (bottom), Row 9 is NORTH (top)
int gridMap[10][10];
// knownMap: true if we've probed this junction and know whether it's an obstacle
bool knownMap[10][10];

// Avoidance stacking: save the original target when obstacleAvoid() is first called
int originalAvoidanceTargetRow = -1;
int originalAvoidanceTargetCol = -1;
Direction originalAvoidanceDirection = NORTH;
bool inAvoidanceMode = false;

// -------------------- Junction detection --------------------
const int WHITE_THRESHOLD = 500;
const int JUNCTION_WHITE_COUNT = 7;
const int TURN_DELAY_MS = 700;
const int TURN_SPEED = 150;
bool junctionDetected = false;

// -------------------- Drop positions --------------------
const int DROP_ROW = 9;
const int BLUE_DROP_COL = 3;
const int GREEN_DROP_COL = 5;
const int RED_DROP_COL = 7;

// ---------------- Encoder Pins ----------------
#define R_ENC_A 18
#define R_ENC_B 19
#define L_ENC_A 2
#define L_ENC_B 3

#define TRIG_PIN 32
#define ECHO_PIN 30

extern volatile long rightPulse;
extern volatile long leftPulse;

// -------- TCS3200 Color Sensor Pins --------
#define S0 39
#define S1 41
#define S2 37
#define S3 35
#define SENSOR_OUT 33

// -------- RGB LED Pins --------
int redPin = 6;
int greenPin = 4;
int bluePin = 5;

// Last detected box color (0=none, 2=RED,3=GREEN,4=BLUE)
int lastDetectedBoxColor = 0;
// When true, delivery logic is active: treat all objects as obstacles and do not pick up boxes
bool inDeliveryMode = false;

const float wheelDiameter = 6.5; // cm
const int encoderPPR = 1650;     // measured pulses per wheel rotation
const float wheelBase = 14;      // distance between wheels in cm (measure your robot)

// -------------------- Function prototypes --------------------
void setMotorsForward(int ls, int rs);
void stopMotors();
void hardLeftTurn90();
void hardRightTurn90();
void turnAround90();
void goDownOneRowFromEnd();
void deliverBoxAndReturn(int color, int savedRow, int savedCol, Direction savedDir);
void moveForward(int acm);
void moveBackward(int acm);
void updatePositionAfterForward();
bool isNextObstacle();
bool box_obstacle();
int detectNextBoxColor();
int detectNextBoxColorMajority(int samples);
void pickUpBox();
void placeBox();
void lineFollowNormal();
void obstacleAvoid();
const char* directionToString(Direction dir);
bool navigateToJunction(int startRow, int startCol, Direction startDir, int targetRow, int targetCol);
int readColor(bool s2State, bool s3State);
String detectColorFromFreq(int r, int g, int b);
int detectBoxAtCurrentPosition();

// -------------------- Flood Fill Pathfinding --------------------
struct Node {
  int row, col;
  int distance;
};

// Simple BFS to find shortest path
bool findPathBFS(int startRow, int startCol, int targetRow, int targetCol, 
                 int path[81][2], int& pathLen) {
  int visited[10][10] = {0};
  Node queue[81];
  int parent[10][10][2];
  
  int queueFront = 0, queueRear = 0;
  
  // Initialize
  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      parent[r][c][0] = -1;
      parent[r][c][1] = -1;
    }
  }
  
  // Start BFS
  queue[queueRear].row = startRow;
  queue[queueRear].col = startCol;
  queue[queueRear].distance = 0;
  queueRear++;
  visited[startRow][startCol] = 1;
  
  while (queueFront < queueRear) {
    Node current = queue[queueFront++];
    
    if (current.row == targetRow && current.col == targetCol) {
      // Found target! Reconstruct path
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
      
      // Reverse path (it's currently target -> start)
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
    
    // Check all 4 directions: NORTH, EAST, SOUTH, WEST
    int dr[] = {1, 0, -1, 0};   // NORTH, EAST, SOUTH, WEST
    int dc[] = {0, 1, 0, -1};
    
    for (int dir = 0; dir < 4; dir++) {
      int nr = current.row + dr[dir];
      int nc = current.col + dc[dir];
      
      // Check bounds and if not visited
      if (nr >= 1 && nr <= 9 && nc >= 1 && nc <= 9 && !visited[nr][nc]) {
        // Prefer map knowledge: if we've probed this junction before, use that info.
        bool canMove = true;
        if (knownMap[nr][nc]) {
          // We know this junction state from prior probing
          if (gridMap[nr][nc] == 1) {
            canMove = false;
          }
        } else {
          // Unknown junction: assume passable (optimistic). The robot will probe when it reaches nearby.
          canMove = true;
        }
        
        // If no obstacles block the path, add to queue
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
  
  return false;  // No path found
}

void calibrateColorSensor() {
  Serial.println("=== TCS3200 COLOR CALIBRATION ===");
  delay(1500);

  // ----------- WHITE Calibration -----------
  Serial.println("Place WHITE surface in front of sensor...");
  delay(3000);
  int wR = readColor(LOW, LOW);
  int wG = readColor(HIGH, HIGH);
  int wB = readColor(LOW, HIGH);
  Serial.print("White R="); Serial.print(wR);
  Serial.print(" G="); Serial.print(wG);
  Serial.print(" B="); Serial.println(wB);

  // ----------- RED Calibration -----------
  Serial.println("Place **RED** object in front of sensor...");
  delay(3000);
  int rR = readColor(LOW, LOW);
  int rG = readColor(HIGH, HIGH);
  int rB = readColor(LOW, HIGH);
  Serial.print("Red R="); Serial.print(rR);
  Serial.print(" G="); Serial.print(rG);
  Serial.print(" B="); Serial.println(rB);

  // ----------- GREEN Calibration -----------
  Serial.println("Place **GREEN** object in front of sensor...");
  delay(3000);
  int gR = readColor(LOW, LOW);
  int gG = readColor(HIGH, HIGH);
  int gB = readColor(LOW, HIGH);
  Serial.print("Green R="); Serial.print(gR);
  Serial.print(" G="); Serial.print(gG);
  Serial.print(" B="); Serial.println(gB);

  // ----------- BLUE Calibration -----------
  Serial.println("Place **BLUE** object in front of sensor...");
  delay(3000);
  int bR = readColor(LOW, LOW);
  int bG = readColor(HIGH, HIGH);
  int bB = readColor(LOW, HIGH);
  Serial.print("Blue R="); Serial.print(bR);
  Serial.print(" G="); Serial.print(bG);
  Serial.print(" B="); Serial.println(bB);

  // ----------- Create Thresholds -----------
  redThresh   = (rR + gR + bR) / 3;   // red pixels show low R frequency
  greenThresh = (gG + rG + bG) / 3;
  blueThresh  = (bB + rB + gB) / 3;

  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.print("Red Threshold   = "); Serial.println(redThresh);
  Serial.print("Green Threshold = "); Serial.println(greenThresh);
  Serial.print("Blue Threshold  = "); Serial.println(blueThresh);
}

// Navigate from current position to target junction using flood fill path
bool navigateToJunction(int startRow, int startCol, Direction startDir, int targetRow, int targetCol) {
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
  
  // Find path using BFS
  if (!findPathBFS(startRow, startCol, targetRow, targetCol, path, pathLen)) {
    Serial.println("ERROR: No path found to target!");
    return false;
  }
  
  Serial.print("Path found with ");
  Serial.print(pathLen);
  Serial.println(" junctions");
  
  // Update current position to start position
  curRow = startRow;
  curCol = startCol;
  currentDirection = startDir;
  
  // Navigate through each junction in the path
  for (int i = 1; i < pathLen; i++) {
    int nextRow = path[i][0];
    int nextCol = path[i][1];
    
    // Determine direction needed to reach next junction
    Direction neededDir;
    if (nextRow > curRow) neededDir = NORTH;
    else if (nextRow < curRow) neededDir = SOUTH;
    else if (nextCol > curCol) neededDir = EAST;
    else neededDir = WEST;
    
    // Turn to face the needed direction
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
    
    // Just-in-time probe: if we don't already know the next junction, probe with ultrasonic
    int probeRow = curRow;
    int probeCol = curCol;
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
        if (obs) {
          Serial.print("Just-in-time probe: obstacle detected at R"); Serial.print(probeRow);
          Serial.print(" C"); Serial.println(probeCol);
          // Abort navigation so caller can re-plan around the newly discovered obstacle
          return false;
        }
      } else {
        // If we already knew it's blocked, abort as well
        if (gridMap[probeRow][probeCol] == 1) {
          Serial.print("Known obstacle ahead at R"); Serial.print(probeRow);
          Serial.print(" C"); Serial.println(probeCol);
          return false;
        }
      }
    }

    // Move forward to next junction using line follow
    lineFollowNormal();
    
    // Update position (lineFollowNormal should handle this)
    Serial.print("Reached R");
    Serial.print(curRow);
    Serial.print(" C");
    Serial.println(curCol);
  }
  
  // Check if target junction has a box — only if not currently delivering
  if (!inDeliveryMode) {
    if (curRow == targetRow && curCol == targetCol) {
      int boxColor = detectBoxAtCurrentPosition();
      if (boxColor != 0) {
        Serial.print("Box detected at target junction! Color: ");
        Serial.println(boxColor);
        // Pick up the box and deliver it
        pickUpBox();
        deliverBoxAndReturn(boxColor, curRow, curCol, currentDirection);
      }
    }
  }
  
  Serial.print("Navigation complete! At R");
  Serial.print(curRow);
  Serial.print(" C");
  Serial.print(curCol);
  Serial.print(" facing ");
  Serial.println(directionToString(currentDirection));
  return true;
}

// -------------------- Utility function for printing --------------------
const char* directionToString(Direction dir) {
  switch (dir) {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case SOUTH: return "SOUTH";
    case WEST: return "WEST";
  }
  return "UNKNOWN";
}

bool isNextObstacle() {
  long duration;
  int distance;

  // Clear trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Send 10µs pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read pulse time
  duration = pulseIn(ECHO_PIN, HIGH, 30000);  // timeout 30ms = ~5m

  if (duration == 0) {
    // No echo received → treat as no obstacle
    return false;
  }

  // Convert to cm
  distance = duration * 0.0343 / 2;

  // Debug
  Serial.print("Distance: ");
  Serial.println(distance);

  // Check threshold
  if (distance <= 13) {
    return true;
  } else {
    return false;
  }
}

void setMotorsForward(int ls, int rs) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, ls);
  analogWrite(ENB, rs);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(400);
}

// -------------------- Turning Functions (Encoder-based) --------------------
void hardRightTurn90() {
  noInterrupts();
  moveForward(2); // Short jog forward for alignment
  int maxSpeed = 150;
  int minSpeed = 80;
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  float wheelCircumference = 3.1416 * wheelDiameter;
  float arcLength = 3.1416 * wheelBase / 4.0;
  long targetPulsesOne = (long)((arcLength / wheelCircumference) * encoderPPR);
  long targetTotal = 2 * targetPulsesOne;

  // PID tuning parameters (tune these for your robot)
  float Kp = 0.4;
  float Ki = 0;
  float Kd = 0;

  float integral = 0.0;
  float lastError = 0.0;
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

    float derivative = 0.0;
    if (dt > 0.0) {
      if (dt > 0.2) dt = 0.2;
      integral += error * dt;

      float maxIntegral = (float)maxSpeed * 2.0;
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;

      derivative = (error - lastError) / dt;
    }
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int turnSpeed = (int)output;
    if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
    if (turnSpeed < 0) turnSpeed = 0;

    if (turnSpeed < minSpeed && error > 10) turnSpeed = minSpeed;

    // Left wheel backwards
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, turnSpeed);

    // Right wheel forwards
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, turnSpeed);
  }
  stopMotors();
  // Update direction after turning 90 degrees right
  switch (currentDirection) {
    case NORTH: currentDirection = EAST; break;
    case EAST:  currentDirection = SOUTH; break;
    case SOUTH: currentDirection = WEST; break;
    case WEST:  currentDirection = NORTH; break;
  }
  Serial.print("Turned RIGHT to face "); Serial.println(directionToString(currentDirection));
}

void hardLeftTurn90() {
  noInterrupts();
  moveForward(2); // Short jog forward for alignment
  int maxSpeed = 150;
  int minSpeed = 80;
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  float wheelCircumference = 3.1416 * wheelDiameter;
  float arcLength = 3.1416 * wheelBase / 4.0;
  long targetPulsesOne = (long)((arcLength / wheelCircumference) * encoderPPR);
  long targetTotal = 2 * targetPulsesOne;

  float Kp = 0.4;
  float Ki = 0.0;
  float Kd = 0.0;

  float integral = 0.0;
  float lastError = 0.0;
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

    float derivative = 0.0;
    if (dt > 0.0) {
      if (dt > 0.2) dt = 0.2;
      integral += error * dt;

      float maxIntegral = (float)maxSpeed * 2.0;
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;

      derivative = (error - lastError) / dt;
    }
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int turnSpeed = (int)output;
    if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
    if (turnSpeed < 0) turnSpeed = 0;

    if (turnSpeed < minSpeed && error > 10) turnSpeed = minSpeed;

    // Left wheel forwards
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, turnSpeed);

    // Right wheel backwards
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, turnSpeed);

    delay(5);
  }
  stopMotors();

  // Update direction after turning 90 degrees left
  switch (currentDirection) {
    case NORTH: currentDirection = WEST; break;
    case EAST:  currentDirection = NORTH; break;
    case SOUTH: currentDirection = EAST; break;
    case WEST:  currentDirection = SOUTH; break;
  }
  Serial.print("Turned LEFT to face "); Serial.println(directionToString(currentDirection));
}

void turnAround90() {
  hardLeftTurn90();
  delay(120);
  hardLeftTurn90();
}

void goDownOneRowFromEnd() {
  if (curRow >= 9) return;

  Serial.println("Going down to next row...");

  // Save the current horizontal direction
  Direction originalHorizontalDir = currentDirection;

  // Step 1: Turn to face NORTH (curRow++)
  if (currentDirection == EAST){
    hardLeftTurn90();
  } else {
    hardRightTurn90();
  }
  // Step 2: Follow vertical line 1 junction (updates curRow)
  lineFollowNormal();

  // Step 3: Turn to face the new horizontal direction (opposite of original)
  if (originalHorizontalDir == EAST) {
    hardLeftTurn90(); // NORTH -> WEST
  } else { // WEST
    hardRightTurn90(); // NORTH -> EAST
  }

  Serial.print("Moved down to row ");
  Serial.print(curRow);
  Serial.print(" col ");
  Serial.print(curCol);
  Serial.print(" facing ");
  Serial.println(directionToString(currentDirection));
}

// ---------- MOVE EXACT DISTANCE (cm) ----------
void moveForward(int acm) {
  int maxSpeed = 150;
  int minSpeed = 80;
  noInterrupts();
  long startRight = rightPulse;
  long startLeft  = leftPulse;
  interrupts();

  float wheelCirc = 3.1416 * wheelDiameter;
  long targetPulses = (long)((acm / wheelCirc) * encoderPPR);

  // ---------- PID ----------
  float Kp = 0.45;
  float Ki = 0.0;
  float Kd = 0.09;

  float integral = 0;
  float lastError = 0;
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
    float maxIntegral = maxSpeed * 2;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int forwardSpeed = (int)output;
    if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
    if (forwardSpeed < 0) forwardSpeed = 0;
    if (forwardSpeed < minSpeed && error > 20) forwardSpeed = minSpeed;

    // ---------- DRIVE FORWARD ----------
    digitalWrite(IN2, HIGH);   // left forward
    digitalWrite(IN1, LOW);
    analogWrite(ENA, forwardSpeed);

    digitalWrite(IN4, HIGH);   // right forward
    digitalWrite(IN3, LOW);
    analogWrite(ENB, forwardSpeed);

    delay(5);
  }

  stopMotors();
}

void moveBackward(int acm) {
  int maxSpeed = 150;
  int minSpeed = 80;
  noInterrupts();
  long startRight = rightPulse;
  long startLeft = leftPulse;
  interrupts();

  float wheelCirc = 3.1416 * wheelDiameter;
  long targetPulses = (long)((acm / wheelCirc) * encoderPPR);

  // ---------- PID ----------
  float Kp = 0.45;
  float Ki = 0.0;
  float Kd = 0.0;

  float integral = 0;
  float lastError = 0;
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
    float maxIntegral = maxSpeed * 2;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int backSpeed = (int)output;
    if (backSpeed > maxSpeed) backSpeed = maxSpeed;
    if (backSpeed < 0) backSpeed = 0;
    if (backSpeed < minSpeed && error > 20) backSpeed = minSpeed;

    // ---------- DRIVE BACKWARD ----------
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, backSpeed);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, backSpeed);

    delay(5);
  }

  stopMotors();
}

// -------------------- POSITION UPDATES --------------------
void updatePositionAfterForward() {
  // Advance curCol/curRow based on the current direction
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

  // The junctionCount tracking is now simplified:
  if (currentDirection == EAST) junctionCount = curCol;
  else if (currentDirection == WEST) junctionCount = 10 - curCol + 1;
  else junctionCount++; // Simplified count for vertical movement
}

// -------------------- Line follow normal --------------------
void lineFollowNormal() {
  float Kp = 0.04;
  float Ki = 0.00;
  float Kd = 0.04;
  long lastError = 0;
  float integral = 0.0;

  while (true) {  // Loop until junction
    unsigned long position = qtr.readLineWhite(sensorValues);

    int whiteCount = 0;
    qtr.readLineWhite(sensorValues); // update raw sensor values (helpful)
    for (int i = 0; i < SensorCount; i++) {
      if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
    }

    // ----------- JUNCTION DETECTION -----------
    if (!junctionDetected) {
      // Only detect if not already detected
      if (whiteCount >= JUNCTION_WHITE_COUNT) {

        junctionDetected = true;   // prevent double count

        digitalWrite(LED_BUILTIN, HIGH);
        stopMotors(); // Stop before coordinate update

        if (autoUpdatePosition) {
          updatePositionAfterForward();  // update curRow, curCol, junctionCount
        }

        // **Requested Output:** Print junction coordinates and direction
        Serial.print("Junction detected at R");
        Serial.print(curRow);
        Serial.print(" C");
        Serial.print(curCol);
        Serial.print(" - Moving ");
        Serial.println(directionToString(currentDirection));

        return;  // Exit on detection
      }
    }

    // ----------- RESET DETECTION WHEN WHITE ENDS -----------
    if (junctionDetected) {
      if (whiteCount < 6) {
        junctionDetected = false;
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    long error = 3500 - position;
    integral += error;
    long derivative = error - lastError;
    lastError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;

    int leftSpeed = constrain((int)(baseSpeed + correction), 0, 255);
    int rightSpeed = constrain((int)(baseSpeed - correction), 0, 255);
    setMotorsForward(leftSpeed, rightSpeed);

    delay(10);
  }
}

// -------------------- Box/obstacle functions (color sensor integration) --------------------
int readColor(bool s2State, bool s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(50);

  int frequency = pulseIn(SENSOR_OUT, LOW);
  return frequency;
}

String detectColorFromFreq(int r, int g, int b) {

  // Detect RED: R is significantly lower than calibrated threshold
  if (r < redThresh && r < g - 20 && r < b - 20)
    return "RED";

  // Detect GREEN
  if (g < greenThresh && g < r - 20 && g < b - 20)
    return "GREEN";

  // Detect BLUE
  if (b < blueThresh && b < r - 20 && b < g - 20)
    return "BLUE";

  // Detect WHITE
  if (r > redThresh && g > greenThresh && b > blueThresh)
    return "WHITE";

  return "UNKNOWN";
}

void setColor(int r, int g, int b) {
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluePin, b);
}

// Majority-sample color detection: take 'samples' readings and pick the majority
int detectNextBoxColorMajority(int samples) {
  int countsRed = 0;
  int countsGreen = 0;
  int countsBlue = 0;
  int countsUnknown = 0;

  Serial.print("Starting majority color sampling (samples="); Serial.print(samples); Serial.println(")");

  for (int i = 0; i < samples; i++) {
    int r = readColor(LOW, LOW);
    int g = readColor(HIGH, HIGH);
    int b = readColor(LOW, HIGH);

    String col = detectColorFromFreq(r, g, b);

    if (col == "RED") countsRed++;
    else if (col == "GREEN") countsGreen++;
    else if (col == "BLUE") countsBlue++;
    else countsUnknown++;

    // Print per-sample debug (optional but useful during tuning)
    Serial.print("Sample "); Serial.print(i+1);
    Serial.print(": R="); Serial.print(r);
    Serial.print(" G="); Serial.print(g);
    Serial.print(" B="); Serial.print(b);
    Serial.print(" -> "); Serial.println(col);

    delay(25); // small pause between samples to let sensor settle
  }

  Serial.print("Color samples counts: RED="); Serial.print(countsRed);
  Serial.print(" GREEN="); Serial.print(countsGreen);
  Serial.print(" BLUE="); Serial.print(countsBlue);
  Serial.print(" UNKNOWN="); Serial.println(countsUnknown);

  // Determine majority
  int maxCount = countsUnknown;
  int colorCode = 0; // 0 = unknown/obstacle
  if (countsRed > maxCount) { maxCount = countsRed; colorCode = 2; }
  if (countsGreen > maxCount) { maxCount = countsGreen; colorCode = 3; }
  if (countsBlue > maxCount) { maxCount = countsBlue; colorCode = 4; }

  // Update LED and lastDetectedBoxColor
  if (colorCode == 2) setColor(255, 0, 0);
  else if (colorCode == 3) setColor(0, 255, 0);
  else if (colorCode == 4) setColor(0, 0, 255);
  else setColor(0, 0, 0);

  lastDetectedBoxColor = colorCode;

  Serial.print("Majority color: ");
  if (colorCode == 2) Serial.println("RED");
  else if (colorCode == 3) Serial.println("GREEN");
  else if (colorCode == 4) Serial.println("BLUE");
  else Serial.println("UNKNOWN/OBSTACLE");

  return colorCode;
}

// Read color and return code: 0=none/unknown/obstacle, 2=RED,3=GREEN,4=BLUE
int detectNextBoxColor() {
  // Use the simpler single-sample method from your working sketch
  // RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(50);
  int r = pulseIn(SENSOR_OUT, LOW);
  if (r == 0) r = 30000;

  // GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(50);
  int g = pulseIn(SENSOR_OUT, LOW);
  if (g == 0) g = 30000;

  // BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(50);
  int b = pulseIn(SENSOR_OUT, LOW);
  if (b == 0) b = 30000;

  Serial.print("R: "); Serial.print(r);
  Serial.print("  G: "); Serial.print(g);
  Serial.print("  B: "); Serial.print(b);
  Serial.print("  =>  ");

  String col = detectColorFromFreq(r, g, b);
  Serial.println(col);

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
  } else if (col == "WHITE") {
    // treat white as non-box (background)
    setColor(255, 255, 255);
    lastDetectedBoxColor = 0;
    return 0;
  } else {
    // UNKNOWN → treat as obstacle
    setColor(0, 0, 0);
    lastDetectedBoxColor = 0;
    return 0;
  }
}

// When asked whether the thing ahead is a box, probe with color sensor
bool box_obstacle() {
  int color = detectNextBoxColor();
  lastDetectedBoxColor = color;
  return (color != 0);
}

// Detect box at current junction (called after arrival)
int detectBoxAtCurrentPosition() {
  int color = detectNextBoxColor();
  return color; // 0=no box, 2=RED,3=GREEN,4=BLUE
}

void pickUpBox() { Serial.println("Pick up box"); delay(500); }
void placeBox() { Serial.println("Place box"); delay(500); }

void deliverBoxAndReturn(int color, int savedRow, int savedCol, Direction savedDir) {
  Serial.print("Delivering box (color="); Serial.print(color); Serial.println(")");

  // Determine drop location based on color (user-specified mapping)
  int dropRow = 3;
  int dropCol = -1;
  if (color == 4) { // BLUE
    dropCol = 2;
  } else if (color == 3) { // GREEN
    dropCol = 4;
  } else if (color == 2) { // RED
    dropCol = 6;
  } else {
    Serial.println("Unknown box color; aborting delivery.");
    return;
  }

  // 1) Pick up the box at current junction
  Serial.println("Picking up box...");
  pickUpBox();
  delay(200);

  // Enter delivery mode: during delivery treat other boxes as obstacles and do not auto-pickup
  inDeliveryMode = true;

  // 2) Navigate to drop location using BFS navigation, but avoid obstacles while en route
  Serial.print("Navigating to drop location R"); Serial.print(dropRow); Serial.print(" C"); Serial.println(dropCol);
  // Retry loop: if a just-in-time probe discovers an obstacle, run avoidance and try again
  int retry = 0;
  const int maxRetries = 8;
  while (!navigateToJunction(curRow, curCol, currentDirection, dropRow, dropCol)) {
    Serial.println("Detected obstacle en-route to drop; invoking avoidance and re-planning...");
    obstacleAvoid();
    retry++;
    if (retry >= maxRetries) {
      Serial.println("ERROR: too many retries navigating to drop location; aborting delivery.");
      inDeliveryMode = false;
      return;
    }
  }

  // 3) Ensure robot faces EAST at drop (as requested)
  Direction needed = EAST;
  while (currentDirection != needed) {
    if ((currentDirection == NORTH && needed == EAST) ||
        (currentDirection == EAST && needed == SOUTH) ||
        (currentDirection == SOUTH && needed == WEST) ||
        (currentDirection == WEST && needed == NORTH)) {
      hardRightTurn90();
    } else {
      hardLeftTurn90();
    }
    delay(100);
  }

  // 4) Place the box
  Serial.println("Placing box at drop location...");
  placeBox();
  delay(200);

  // 5) Navigate back to saved pickup junction, avoiding obstacles as needed
  Serial.print("Returning to pickup junction R"); Serial.print(savedRow); Serial.print(" C"); Serial.println(savedCol);
  retry = 0;
  while (!navigateToJunction(curRow, curCol, currentDirection, savedRow, savedCol)) {
    Serial.println("Detected obstacle en-route to pickup; invoking avoidance and re-planning...");
    obstacleAvoid();
    retry++;
    if (retry >= maxRetries) {
      Serial.println("WARNING: too many retries returning to pickup; aborting return.");
      inDeliveryMode = false;
      return;
    }
  }

  // 6) Restore facing direction
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

  Serial.print("Returned to R"); Serial.print(curRow); Serial.print(" C"); Serial.print(curCol);
  Serial.print(" facing "); Serial.println(directionToString(currentDirection));
  // Exit delivery mode
  inDeliveryMode = false;
}

void obstacleAvoid() {
  Serial.println("Avoiding obstacle (BFS-assisted) ...");

  Direction originalDirection = currentDirection;

  // If this is the first avoidance call, mark entry into avoidance mode
  if (!inAvoidanceMode) {
    inAvoidanceMode = true;
    originalAvoidanceDirection = originalDirection;
  }

  while (true) {  // Loop until junction
    unsigned long position = qtr.readLineWhite(sensorValues);
    moveBackward(2);

    int whiteCount = 0;
    qtr.readLineWhite(sensorValues); // update raw sensor values (helpful)
    for (int i = 0; i < SensorCount; i++) {
      if (sensorValues[i] < WHITE_THRESHOLD) whiteCount++;
    }

    // ----------- JUNCTION DETECTION -----------
    junctionDetected = false;
    if (!junctionDetected) {
      // Only detect if not already detected
      if (whiteCount >= JUNCTION_WHITE_COUNT) {

        junctionDetected = true;   // prevent double count

        digitalWrite(LED_BUILTIN, HIGH);
        stopMotors(); // Stop before coordinate update

        // **Requested Output:** Print junction coordinates and direction
        Serial.print("Junction detected at R");
        Serial.print(curRow);
        Serial.print(" C");
        Serial.print(curCol);
        Serial.print(" - Moving ");
        Serial.println(directionToString(currentDirection));

        break;  // Exit on detection
      }
    }
  }

  Serial.print("Backed up to R"); Serial.print(curRow); Serial.print(" C"); Serial.println(curCol);

  // 2) Compute desired skip target (two junctions ahead in original direction)
  int targetRow = curRow;
  int targetCol = curCol;
  switch (originalDirection) {
    case EAST: targetCol = curCol + 2; break;
    case WEST: targetCol = curCol - 2; break;
    case NORTH: targetRow = curRow + 2; break;
    case SOUTH: targetRow = curRow - 2; break;
  }

  // Clamp target to valid range
  if (targetRow < 1) targetRow = 1;
  if (targetRow > 9) targetRow = 9;
  if (targetCol < 1) targetCol = 1;
  if (targetCol > 9) targetCol = 9;

  // 3) Try BFS to reach the skip target. If path exists, navigate there.
  int path[81][2];
  int pathLen = 0;

  if (gridMap[targetRow][targetCol] != 1) {
    bool found = findPathBFS(curRow, curCol, targetRow, targetCol, path, pathLen);
    if (found && pathLen > 0) {
      Serial.print("BFS path to skip-target found (length="); Serial.print(pathLen); Serial.println(")");
      // Save the first avoidance target if not already saved
      if (originalAvoidanceTargetRow == -1) {
        originalAvoidanceTargetRow = targetRow;
        originalAvoidanceTargetCol = targetCol;
      }
      // Use navigateToJunction which will follow path and update curRow/curCol/currentDirection
      if (navigateToJunction(curRow, curCol, currentDirection, targetRow, targetCol)) {
        gridMap[curRow][curCol] = 0;
        // Now navigate back to the original target junction
        Serial.print("Avoidance intermediate complete. Returning to original target R"); 
        Serial.print(originalAvoidanceTargetRow); Serial.print(" C"); Serial.println(originalAvoidanceTargetCol);
        if (navigateToJunction(curRow, curCol, currentDirection, originalAvoidanceTargetRow, originalAvoidanceTargetCol)) {
          // Restore facing to the original direction
          while (currentDirection != originalDirection) {
            if ((currentDirection == NORTH && originalDirection == EAST) ||
                (currentDirection == EAST && originalDirection == SOUTH) ||
                (currentDirection == SOUTH && originalDirection == WEST) ||
                (currentDirection == WEST && originalDirection == NORTH)) {
              hardRightTurn90();
            } else {
              hardLeftTurn90();
            }
          }
          // Reset avoidance mode
          inAvoidanceMode = false;
          originalAvoidanceTargetRow = -1;
          originalAvoidanceTargetCol = -1;
          return;
        } else {
          Serial.println("Failed to return to original target.");
          inAvoidanceMode = false;
          originalAvoidanceTargetRow = -1;
          originalAvoidanceTargetCol = -1;
          return;
        }
      } else {
        Serial.println("Navigation aborted during skip-target traversal; will try further candidates.");
      }
    }
  }

  // 4) If direct skip-target not reachable, try to find a reachable junction BEYOND the obstacle
  Serial.println("Direct skip-target not reachable; searching line-ahead for junction beyond obstacle...");
  int searchRow = curRow;
  int searchCol = curCol;
  bool reached = false;

  // Start search from step=2 to ensure we go PAST the immediate obstacle, not just around it
  for (int step = 2; step <= 8; step++) {
    switch (originalDirection) {
      case EAST: searchCol = curCol + step; searchRow = curRow; break;
      case WEST: searchCol = curCol - step; searchRow = curRow; break;
      case NORTH: searchRow = curRow + step; searchCol = curCol; break;
      case SOUTH: searchRow = curRow - step; searchCol = curCol; break;
    }
    if (searchRow < 1 || searchRow > 9 || searchCol < 1 || searchCol > 9) break;
    if (gridMap[searchRow][searchCol] == 1) {
      Serial.print("Step "); Serial.print(step); Serial.print(": (R"); Serial.print(searchRow);
      Serial.print(",C"); Serial.print(searchCol); Serial.println(") blocked, continuing...");
      continue; // still blocked
    }

    // attempt BFS to this candidate (must successfully navigate AND verify path clears obstacle)
    pathLen = 0;
    if (findPathBFS(curRow, curCol, searchRow, searchCol, path, pathLen)) {
      Serial.print("Found BFS path to R"); Serial.print(searchRow); Serial.print(" C"); Serial.print(searchCol);
      Serial.print(" (len="); Serial.print(pathLen); Serial.println(")");
      if (navigateToJunction(curRow, curCol, currentDirection, searchRow, searchCol)) {
        gridMap[curRow][curCol] = 0;
        // Save the first successful avoidance target if not already saved
        if (originalAvoidanceTargetRow == -1) {
          originalAvoidanceTargetRow = searchRow;
          originalAvoidanceTargetCol = searchCol;
        }
        // Now navigate back to the original target junction
        Serial.print("Avoidance intermediate complete (fallback). Returning to original target R"); 
        Serial.print(originalAvoidanceTargetRow); Serial.print(" C"); Serial.println(originalAvoidanceTargetCol);
        if (navigateToJunction(curRow, curCol, currentDirection, originalAvoidanceTargetRow, originalAvoidanceTargetCol)) {
          // Restore facing to the original direction
          while (currentDirection != originalDirection) {
            if ((currentDirection == NORTH && originalDirection == EAST) ||
                (currentDirection == EAST && originalDirection == SOUTH) ||
                (currentDirection == SOUTH && originalDirection == WEST) ||
                (currentDirection == WEST && originalDirection == NORTH)) {
              hardRightTurn90();
            } else {
              hardLeftTurn90();
            }
          }
          // Reset avoidance mode
          inAvoidanceMode = false;
          originalAvoidanceTargetRow = -1;
          originalAvoidanceTargetCol = -1;
          reached = true;
          break;
        } else {
          Serial.println("Failed to return to original target.");
          inAvoidanceMode = false;
          originalAvoidanceTargetRow = -1;
          originalAvoidanceTargetCol = -1;
          reached = true;
          break;
        }
      } else {
        Serial.println("Navigation aborted during candidate traversal; continuing search...");
      }
    } else {
      Serial.print("BFS to R"); Serial.print(searchRow); Serial.print(" C"); Serial.println(searchCol);
      Serial.println(" failed (likely edge obstacle blocking)");
    }
  }

  if (reached) return;

  // 5) As a last resort, back out and report failure so main loop can re-evaluate
  Serial.println("No BFS path found around obstacle; backing out and aborting avoidance.");
  moveBackward(8);
  switch (originalDirection) {
    case EAST: curCol--; break;
    case WEST: curCol++; break;
    case SOUTH: curRow++; break;
    case NORTH: curRow--; break;
  }
  Serial.print("Backed out to R"); Serial.print(curRow); Serial.print(" C"); Serial.println(curCol);
  // Reset avoidance mode on failure
  inAvoidanceMode = false;
  originalAvoidanceTargetRow = -1;
  originalAvoidanceTargetCol = -1;
  return;
}

// ================== MAIN TASK1 ENTRY FUNCTION ==================
// Call this from main.cpp's runTask1() to run the monolithic grid solver

bool task1_monolithic_run() {
  static bool initialized = false;

  if (!initialized) {
    Serial.println("[Task1_Monolithic] Initializing grid solver...");
    
    // Initialize pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(SENSOR_OUT, INPUT);
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    
    // Set color sensor to 20% frequency
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

    // Initialize grid map
    for (int r = 1; r <= 9; r++) {
      for (int c = 1; c <= 9; c++) {
        gridMap[r][c] = 0;
        knownMap[r][c] = false;
      }
    }
    gridMap[1][4] = 1;  // Known obstacle at R1, C4

    // Set up QTR
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, SensorCount);

    curRow = 1;
    curCol = 0;
    currentDirection = EAST;

    // Calibrate color sensor
    calibrateColorSensor();

    // Move to first junction (1,1) using line-following
    Serial.println("[Task1_Monolithic] Moving to first junction (1,1)...");
    lineFollowNormal();
    gridMap[curRow][curCol] = 0;  // Mark starting junction as empty

    initialized = true;
    return false;  // Still running
  }

  // Main grid scanning loop
  // Check end of row FIRST (prevents overrun)
  bool atRowEnd = (currentDirection == EAST && curCol == 9) || (currentDirection == WEST && curCol == 1);
  if (atRowEnd) {
    if (curRow < 9) {
      goDownOneRowFromEnd();
      gridMap[curRow][curCol] = 0;  // Mark new row-start as empty
    } else {
      Serial.println("[Task1_Monolithic] Finished scanning all rows! Task complete.");
      stopMotors();
      return true;  // Task complete
    }
    return false;  // Continue
  }

  bool nextHasObstacle = isNextObstacle();

  // Mark the probed junction in the map/knownMap so BFS can use prior knowledge
  int probeRow = curRow;
  int probeCol = curCol;
  switch(currentDirection) {
    case EAST: probeCol++; break;
    case WEST: probeCol--; break;
    case NORTH: probeRow++; break;
    case SOUTH: probeRow--; break;
  }
  if (!(probeRow < 1 || probeRow > 9 || probeCol < 1 || probeCol > 9)) {
    if (nextHasObstacle) {
      gridMap[probeRow][probeCol] = 1;
      knownMap[probeRow][probeCol] = true;
      Serial.print("Marked known obstacle at R"); Serial.print(probeRow); Serial.print(" C"); Serial.println(probeCol);
    } else {
      // No obstacle detected at probe location — remember it's free
      knownMap[probeRow][probeCol] = true;
      gridMap[probeRow][probeCol] = 0;
    }
  }

  if (!nextHasObstacle) {
    // --- Step 2: Normal line-following move to next junction ---
    lineFollowNormal();            // PID line-follow until next junction (updates position)
    gridMap[curRow][curCol] = 0;
  } else {
    // --- Step 3: Obstacle detected ahead ---
    Serial.println("[Task1_Monolithic] Obstacle detected ahead. Moving forward 4cm to inspect...");
    moveForward(4);

    // Now check the color of the object using majority sampling
    int color = detectNextBoxColorMajority(10);
    
    // If color is RED (2), GREEN (3), or BLUE (4), it's a box
    if (color == 2 || color == 3 || color == 4) {
      Serial.print("[Task1_Monolithic] Box detected! Color code: "); Serial.println(color);
      Serial.println("[Task1_Monolithic] Picking up box and navigating to delivery location...");
      pickUpBox();
      deliverBoxAndReturn(color, curRow, curCol, currentDirection);
    } else {
      // Not a box color — it's a true obstacle
      Serial.println("[Task1_Monolithic] Not a box. Marking as obstacle and executing avoidance.");
      
      // Mark the next junction (in the direction we're facing) as an obstacle
      int obsRow = curRow;
      int obsCol = curCol;
      switch(currentDirection) {
        case EAST: obsCol++; break;
        case WEST: obsCol--; break;
        case NORTH: obsRow++; break;
        case SOUTH: obsRow--; break;
      }
      if (!(obsRow < 1 || obsRow > 9 || obsCol < 1 || obsCol > 9)) {
        gridMap[obsRow][obsCol] = 1;
        knownMap[obsRow][obsCol] = true;
        Serial.print("[Task1_Monolithic] Marked obstacle at R"); Serial.print(obsRow); Serial.print(" C"); Serial.println(obsCol);
      }
      
      // Execute avoidance to navigate around the obstacle
      obstacleAvoid();
      gridMap[curRow][curCol] = 0;  // Mark the current junction as empty after avoidance
    }
  }

  delay(60); // small loop delay
  return false;  // Continue running
}
