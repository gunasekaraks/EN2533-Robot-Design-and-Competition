#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  // Constructor: pass motor pins
  // Left motor: ena, in1, in2
  // Right motor: enb, in3, in4
  MotorControl(int ena, int in1, int in2, int enb, int in3, int in4);

  // Initialize motor pins (call in setup)
  void begin();

  // Stop both motors immediately
  void stop();

  // Move forward at given speed (0-255)
  void forward(int speed);

  // Move backward at given speed (0-255)
  void backward(int speed);

  // Turn right in place (left motor forward, right motor backward)
  void turnRight(int speed);

  // Turn left in place (left motor backward, right motor forward)
  void turnLeft(int speed);

  // Stop left motor only
  void stopLeft();

  // Stop right motor only
  void stopRight();

  // Set individual motor speeds
  void setLeftMotor(int speed, bool forward = true);
  void setRightMotor(int speed, bool forward = true);

private:
  int ena, in1, in2, enb, in3, in4;
  void setMotor(int en, int in_a, int in_b, int speed, bool forward);
};

#endif // MOTORCONTROL_H
