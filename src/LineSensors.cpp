#include "LineSensors.h"

LineSensors::LineSensors(const uint8_t *pins, uint8_t count) {
  if (count > MAX_SENSORS) count = MAX_SENSORS;
  sensorCount = count;
  for (uint8_t i = 0; i < sensorCount; i++) sensorPins[i] = pins[i];
}

void LineSensors::begin() {
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, sensorCount);
}

void LineSensors::autoCalibrate(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    qtr.calibrate();
    delay(10);
  }
}

void LineSensors::readRaw(uint16_t *out) {
  qtr.read(out);
}

unsigned long LineSensors::readLineWhite(uint16_t *out) {
  return qtr.readLineWhite(out);
}
