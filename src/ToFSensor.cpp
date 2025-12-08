#include "ToFSensor.h"

ToFSensor::ToFSensor(uint8_t xshutPin)
  : xshut(xshutPin), address(0) {}

bool ToFSensor::begin(uint8_t newAddress) {
  address = newAddress;
  pinMode(xshut, OUTPUT);

  // Put sensor into reset
  digitalWrite(xshut, LOW);
  delay(10);

  // Enable sensor and initialize
  digitalWrite(xshut, HIGH);
  delay(10);

  if (!lox.begin()) {
    return false;
  }

  // Set a new I2C address so multiple sensors can coexist
  lox.setAddress(address);
  delay(10);
  return true;
}

int ToFSensor::readRangeMM() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return (int)measure.RangeMilliMeter;
  }
  return -1; // out of range or error
}
