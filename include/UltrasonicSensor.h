#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
  // Constructor: pass trigger and echo pins
  UltrasonicSensor(int trigPin, int echoPin);

  // Initialize pins (call in setup)
  void begin();

  // Get distance in cm
  // Returns -1 if no echo received (timeout)
  int getDistance();

  // Check if object is closer than threshold (default 28 cm)
  bool isObstacleDetected(int thresholdCm = 28);

private:
  int trigPin;
  int echoPin;
  static const long TIMEOUT_MICROS = 30000;  // 30ms timeout = ~5m max range
  static const float SPEED_OF_SOUND = 0.0343; // cm per microsecond
};

#endif // ULTRASONIC_SENSOR_H
