#include "ColorSensor.h"
#include "RGBLED.h"

ColorSensor::ColorSensor(int s0_, int s1_, int s2_, int s3_, int sensorOut_, RGBLED *rgbLed_)
  : s0(s0_), s1(s1_), s2(s2_), s3(s3_), sensorOut(sensorOut_), rgbLed(rgbLed_) {}

void ColorSensor::begin() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
}

int ColorSensor::readFrequency(bool s2State, bool s3State) {
  digitalWrite(s2, s2State);
  digitalWrite(s3, s3State);
  delay(50);
  int frequency = pulseIn(sensorOut, LOW);
  return frequency;
}

void ColorSensor::readRGB(int &red, int &green, int &blue) {
  red = readFrequency(LOW, LOW);      // Red filter
  green = readFrequency(HIGH, HIGH);  // Green filter
  blue = readFrequency(LOW, HIGH);    // Blue filter
}

String ColorSensor::detectColor(int r, int g, int b) {
  String detectedColor = "UNKNOWN";

  // WHITE: R=189, G=192, B=171
  if (r < 240 && g < 240 && b < 240) {
    detectedColor = "WHITE";
  }
  // RED: R=231 (R is lowest)
  else if (r < 280 && r < g - 50 && r < b - 40) {
    detectedColor = "RED";
  }
  // BLUE: R=349, G=312, B=221 (B is lowest)
  else if (b < 260 && b < r - 50 && b < g - 80) {
    detectedColor = "BLUE";
  }
  // GREEN: R=489, G=391, B=410 (G is lowest)
  else if (g < r - 40 && g < b - 20) {
    detectedColor = "GREEN";
  }

  // Update RGB LED if available
  if (rgbLed != nullptr) {
    if (detectedColor == "RED") {
      rgbLed->red();
    } else if (detectedColor == "GREEN") {
      rgbLed->green();
    } else if (detectedColor == "BLUE") {
      rgbLed->blue();
    } else if (detectedColor == "WHITE") {
      rgbLed->white();
    } else {
      rgbLed->off();
    }
  }

  return detectedColor;
}

String ColorSensor::readColor() {
  int r, g, b;
  readRGB(r, g, b);
  return detectColor(r, g, b);
}

void ColorSensor::setRGBLED(RGBLED *rgbLed_) {
  rgbLed = rgbLed_;
}
