#ifndef RGBLED_H
#define RGBLED_H

#include <Arduino.h>

class RGBLED {
public:
  // Constructor with PWM pins for red, green, blue
  RGBLED(int redPin, int greenPin, int bluePin);

  // Initialize pins (call in setup)
  void begin();

  // Set color using RGB values (0-255 each)
  void setColor(int red, int green, int blue);

  // Preset colors
  void red();
  void green();
  void blue();
  void white();
  void yellow();
  void cyan();
  void magenta();
  void orange();
  void off();

private:
  int redPin, greenPin, bluePin;
};

#endif // RGBLED_H
