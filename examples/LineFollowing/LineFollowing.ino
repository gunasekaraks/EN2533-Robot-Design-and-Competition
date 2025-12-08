#include <QTRSensors.h>

// Line-following example using QTR RC sensor array.
// This sketch performs a short auto-calibration at startup (move/sweep the
// sensors across the line while it runs) and then follows the line using a PID controller.

QTRSensors qtr;
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {52, 50, 48, 46, 44, 42, 40, 38};
uint16_t sensorValues[SensorCount];

// Motor pins (match your wiring)
const int ENA = 10, IN1 = 22, IN2 = 24;  // Left motor
const int ENB = 11, IN3 = 26, IN4 = 28;  // Right motor

// PID settings (tune for your robot)
float Kp = 0.20;
float Ki = 0.00;
float Kd = 0.05;

int baseSpeed = 120;  // Base motor speed (0-255)
long lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // QTR setup
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  // Auto-calibration: move/sweep the sensors across the line while this runs
  Serial.println("Auto-calibrating sensors for ~3 seconds. Move sensors across the line now.");
  for (uint16_t i = 0; i < 300; i++) {
    qtr.calibrate();
    delay(10);
  }
  Serial.println("Calibration done.");
}

void loop() {
  // Read line position (0 = far left, 7000 = far right)
  unsigned long position = qtr.readLineWhite(sensorValues);

  // Calculate error relative to center
  long error = 3500 - (long)position; // 3500 = center for 8 sensors

  // PID calculations
  integral += error;
  long derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Calculate motor speeds
  int leftSpeed  = baseSpeed + (int)correction;
  int rightSpeed = baseSpeed - (int)correction;

  // Constrain motor speeds to forward range
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Set left motor (forward)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, leftSpeed);

  // Set right motor (forward)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, rightSpeed);

  // Optional: small delay for stability
  delay(5);
}
