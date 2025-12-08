#include "RGBLED.h"

RGBLED::RGBLED(int redPin_, int greenPin_, int bluePin_)
  : redPin(redPin_), greenPin(greenPin_), bluePin(bluePin_) {}

void RGBLED::begin() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  off();
}

void RGBLED::setColor(int red, int green, int blue) {
  analogWrite(redPin, constrain(red, 0, 255));
  analogWrite(greenPin, constrain(green, 0, 255));
  analogWrite(bluePin, constrain(blue, 0, 255));
}

void RGBLED::red() {
  setColor(255, 0, 0);
}

void RGBLED::green() {
  setColor(0, 255, 0);
}

void RGBLED::blue() {
  setColor(0, 0, 255);
}

void RGBLED::white() {
  setColor(255, 255, 255);
}

void RGBLED::yellow() {
  setColor(255, 255, 0);
}

void RGBLED::cyan() {
  setColor(0, 255, 255);
}

void RGBLED::magenta() {
  setColor(255, 0, 255);
}

void RGBLED::orange() {
  setColor(255, 80, 0);
}

void RGBLED::off() {
  setColor(0, 0, 0);
}
