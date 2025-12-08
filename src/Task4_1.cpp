#include "Task4_1.h"
#include "LineSensors.h"
#include "LineFollower.h"
#include "UltrasonicSensor.h"
#include "CircleFollower.h"
#include "TurnController.h"
#include "MotorControl.h"

// Extern motor/encoder symbols used by TurnController
extern volatile long rightPulse;
extern volatile long leftPulse;
extern const int ENA;
extern const int IN1;
extern const int IN2;
extern const int ENB;
extern const int IN3;
extern const int IN4;

Task4_1::Task4_1(LineSensors* qtr_, LineFollower* follower_, UltrasonicSensor* us_, CircleFollower* circle_,
                 float wheelDiameterCm_, int encoderPPR_, float wheelBaseCm_, int turnMaxSpeed_, int turnMinSpeed_,
                 int ultrasonicThresholdCm_)
  : qtr(qtr_), follower(follower_), us(us_), circle(circle_),
    wheelDiameterCm(wheelDiameterCm_), encoderPPR(encoderPPR_), wheelBaseCm(wheelBaseCm_),
    turnMaxSpeed(turnMaxSpeed_), turnMinSpeed(turnMinSpeed_), ultrasonicThresholdCm(ultrasonicThresholdCm_),
    started(false), state(FOLLOW_LINE) {}

void Task4_1::begin() {
  started = true;
  state = FOLLOW_LINE;
}

bool Task4_1::update() {
  if (!started) return false;

  switch (state) {
    case FOLLOW_LINE: {
      // read sensors and follow line
      uint8_t cnt = qtr->count();
      uint16_t vals[16]; // supports up to 16
      if (cnt > 16) cnt = 16;
      unsigned long pos = qtr->readLineWhite(vals);
      follower->update(pos);

      // check ultrasonic
      if (us) {
        int d = us->getDistance();
        if (d > 0 && d < ultrasonicThresholdCm) {
          Serial.print("Task4_1: obstacle detected at "); Serial.print(d); Serial.println(" cm -> turning left");
          follower->stop();
          // perform left 90 degree turn (negative angle)
          turnAngle(-90.0f, wheelDiameterCm, encoderPPR, wheelBaseCm, turnMaxSpeed, turnMinSpeed);
          // start circle follower
          if (circle) {
            circle->begin();
          }
          state = RUN_CIRCLE;
        }
      }
      break;
    }

    case RUN_CIRCLE: {
      if (circle) {
        FollowMode m = circle->update();
        // consider task complete when inner circle following begins
        if (m == INNER_CIRCLE) {
          Serial.println("Task4_1: Circle follower reached INNER_CIRCLE -> task complete");
          state = COMPLETE;
          return true;
        }
      }
      break;
    }

    case COMPLETE:
      return true;

    default:
      break;
  }

  return (state == COMPLETE);
}
