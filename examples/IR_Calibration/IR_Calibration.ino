#include <QTRSensors.h>

// IR array calibration sketch
// - Move the robot/sensor array across the line/surfaces while this runs
// - It prints per-sensor min/max raw RC counts which you can copy into
//   your line-following sketch or use to inspect sensor behavior.

QTRSensors qtr;
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {52, 50, 48, 46, 44, 42, 40, 38};
uint16_t sensorValues[SensorCount];
uint16_t minVals[SensorCount];
uint16_t maxVals[SensorCount];

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  // Initialize min/max
  for (uint8_t i = 0; i < SensorCount; i++) {
    minVals[i] = 0xFFFF;
    maxVals[i] = 0;
  }

  Serial.println();
  Serial.println("IR sensor array calibration");
  Serial.println("Move the robot or sweep sensors across the line for ~6 seconds.");
  Serial.println("Calibration running...");

  // Collect samples for a short period while user moves the sensors
  unsigned long start = millis();
  const unsigned long CAL_TIME_MS = 6000;
  while (millis() - start < CAL_TIME_MS) {
    qtr.read(sensorValues); // raw RC counts
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] < minVals[i]) minVals[i] = sensorValues[i];
      if (sensorValues[i] > maxVals[i]) maxVals[i] = sensorValues[i];
    }
    delay(20);
  }

  Serial.println("Calibration complete. Raw min,max per sensor:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(i);
    Serial.print(": min="); Serial.print(minVals[i]);
    Serial.print(", max="); Serial.println(maxVals[i]);
  }

  Serial.println();
  Serial.println("Notes:");
  Serial.println(" - Higher raw RC count usually means darker (more reflectance) depending on wiring.");
  Serial.println(" - Copy the arrays above into your line-following sketch, or run that sketch's auto-calibration.");
}

void loop() {
  // Nothing to do here after calibration; keep serial open for user to copy values.
}
