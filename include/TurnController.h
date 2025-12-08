#ifndef TURNCONTROLLER_H
#define TURNCONTROLLER_H

#include <Arduino.h>

// The implementation expects the following symbols to exist in the sketch (main.cpp):
// - motor pin constants: ENA, IN1, IN2, ENB, IN3, IN4
// - encoder pulse counters: extern volatile long rightPulse; extern volatile long leftPulse;

extern volatile long rightPulse;
extern volatile long leftPulse;

// Motor pin constants (defined in main.cpp)
extern const int ENA;
extern const int IN1;
extern const int IN2;
extern const int ENB;
extern const int IN3;
extern const int IN4;

// Stop both motors
void stopMotorsTC();

// Turn the robot in place by a given angle (degrees).
// Positive angle -> turn right (clockwise looking from above).
// Negative angle -> turn left.
// Parameters:
//  - angleDeg: degrees to turn (positive = right)
//  - wheelDiameterCm: wheel diameter in cm
//  - encoderPPR: pulses per wheel rotation (measured)
//  - wheelBaseCm: distance between wheels in cm
//  - maxSpeed: max PWM (0-255)
//  - minSpeed: minimum PWM used while approaching target
void turnAngle(float angleDeg, float wheelDiameterCm, int encoderPPR, float wheelBaseCm, int maxSpeed, int minSpeed);

#endif // TURNCONTROLLER_H
