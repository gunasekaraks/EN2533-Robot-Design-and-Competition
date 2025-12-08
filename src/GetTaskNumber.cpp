#include "GetTaskNumber.h"

GetTaskNumber::GetTaskNumber(uint8_t dip1Pin_, uint8_t dip2Pin_, uint8_t dip3Pin_)
  : dip1Pin(dip1Pin_), dip2Pin(dip2Pin_), dip3Pin(dip3Pin_) {}

void GetTaskNumber::begin() {
  pinMode(dip1Pin, INPUT_PULLUP);
  pinMode(dip2Pin, INPUT_PULLUP);
  pinMode(dip3Pin, INPUT_PULLUP);
}

uint8_t GetTaskNumber::readTask() {
  int b1 = digitalRead(dip1Pin); // LSB
  int b2 = digitalRead(dip2Pin);
  int b3 = digitalRead(dip3Pin); // MSB

  // Invert because ON = LOW on the DIP (active-low)
  b1 = !b1;
  b2 = !b2;
  b3 = !b3;

  uint8_t task = ( (b3 << 2) | (b2 << 1) | (b1 << 0) );
  return task;
}
