#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>

class RGBLED;  // Forward declaration

class ColorSensor {
public:
  // Constructor: pass S0, S1, S2, S3 (frequency scaling), and OUT (sensor output) pins
  // Optional: rgbLed pointer to control RGB LED based on detected color
  ColorSensor(int s0, int s1, int s2, int s3, int sensorOut, RGBLED *rgbLed = nullptr);

  // Initialize pins and set frequency scaling to 20% (HIGH/LOW)
  void begin();

  // Read raw frequency values for R, G, B channels
  void readRGB(int &red, int &green, int &blue);

  // Detect color based on thresholds (returns "RED", "GREEN", "BLUE", "WHITE", or "UNKNOWN")
  String detectColor(int red, int green, int blue);

  // Convenience method: read RGB and return detected color string (also updates RGB LED if set)
  String readColor();

  // Set RGB LED pointer (optional)
  void setRGBLED(RGBLED *rgbLed);

private:
  int s0, s1, s2, s3, sensorOut;
  RGBLED *rgbLed;
  int readFrequency(bool s2State, bool s3State);
};

#endif // COLORSENSOR_H
