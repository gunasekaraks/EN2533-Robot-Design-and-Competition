#ifndef TOFSENSOR_H
#define TOFSENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

class ToFSensor {
public:
  // Constructor with XSHUT pin (required when using multiple VL53L0X on same I2C bus)
  ToFSensor(uint8_t xshutPin);

  // Initialize sensor and assign a new I2C address (call after Wire.begin())
  // Returns true on success
  bool begin(uint8_t newAddress);

  // Read range in millimeters, or -1 if invalid
  int readRangeMM();

  // Raw access to underlying Adafruit object if needed
  Adafruit_VL53L0X* getSensor() { return &lox; }

private:
  uint8_t xshut;
  Adafruit_VL53L0X lox;
  uint8_t address;
};

#endif // TOFSENSOR_H
