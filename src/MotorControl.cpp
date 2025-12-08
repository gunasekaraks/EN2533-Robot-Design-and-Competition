#include "MotorControl.h"

MotorControl::MotorControl(int ena_, int in1_, int in2_, int enb_, int in3_, int in4_)
  : ena(ena_), in1(in1_), in2(in2_), enb(enb_), in3(in3_), in4(in4_) {}

void MotorControl::begin() {
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  stop();
}

void MotorControl::setMotor(int en, int in_a, int in_b, int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  analogWrite(en, speed);
  if (forward) {
    digitalWrite(in_a, LOW);
    digitalWrite(in_b, HIGH);
  } else {
    digitalWrite(in_a, HIGH);
    digitalWrite(in_b, LOW);
  }
}

void MotorControl::stop() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}

void MotorControl::forward(int speed) {
  setMotor(ena, in1, in2, speed, true);
  setMotor(enb, in3, in4, speed, true);
}

void MotorControl::backward(int speed) {
  setMotor(ena, in1, in2, speed, false);
  setMotor(enb, in3, in4, speed, false);
}

void MotorControl::turnRight(int speed) {
  setMotor(ena, in1, in2, speed, true);   // left forward
  setMotor(enb, in3, in4, speed, false);  // right backward
}

void MotorControl::turnLeft(int speed) {
  setMotor(ena, in1, in2, speed, false);  // left backward
  setMotor(enb, in3, in4, speed, true);   // right forward
}

void MotorControl::stopLeft() {
  analogWrite(ena, 0);
}

void MotorControl::stopRight() {
  analogWrite(enb, 0);
}

void MotorControl::setLeftMotor(int speed, bool forward) {
  setMotor(ena, in1, in2, speed, forward);
}

void MotorControl::setRightMotor(int speed, bool forward) {
  setMotor(enb, in3, in4, speed, forward);
}
