#ifndef LINESENSORS_H
#define LINESENSORS_H

#include <Arduino.h>
#include <QTRSensors.h>

class LineSensors {
public:
  static const uint8_t MAX_SENSORS = 16;

  // pins: pointer to array of sensor pins, count: number of sensors (<= MAX_SENSORS)
  LineSensors(const uint8_t *pins, uint8_t count);

  // Initialize hardware (call in setup)
  void begin();

  // Auto-calibrate sensors for durationMs milliseconds (call while sweeping sensors across surface)
  void autoCalibrate(unsigned long durationMs);

  // Read raw RC counts into out[] (caller must allocate >= count())
  void readRaw(uint16_t *out);

  // Read calibrated line position (wraps QTRSensors::readLineWhite)
  unsigned long readLineWhite(uint16_t *out);

  uint8_t count() const { return sensorCount; }

private:
  QTRSensors qtr;
  uint8_t sensorPins[MAX_SENSORS];
  uint8_t sensorCount;
};

#endif // LINESENSORS_H
