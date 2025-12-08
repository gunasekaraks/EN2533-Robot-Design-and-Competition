#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin)
  : trigPin(trigPin), echoPin(echoPin) {}

void UltrasonicSensor::begin() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int UltrasonicSensor::getDistance() {
  long duration;
  int distance;

  // Clear trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Send 10µs pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read pulse time
  duration = pulseIn(echoPin, HIGH, TIMEOUT_MICROS);

  if (duration == 0) {
    // No echo received → timeout
    return -1;
  }

  // Convert to cm: distance = (duration / 2) * speed_of_sound
  distance = duration * SPEED_OF_SOUND / 2;

  return distance;
}

bool UltrasonicSensor::isObstacleDetected(int thresholdCm) {
  int dist = getDistance();
  
  // If no valid reading, treat as no obstacle
  if (dist < 0) {
    return false;
  }

  // Debug output
  Serial.print("Ultrasonic Distance: ");
  Serial.print(dist);
  Serial.println(" cm");

  // Check threshold
  return (dist < thresholdCm);
}
