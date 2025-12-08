#ifndef GET_TASK_NUMBER_H
#define GET_TASK_NUMBER_H

#include <Arduino.h>

// Simple helper to read a 3-bit DIP switch and return task number 0..7
class GetTaskNumber {
public:
  // pin order: LSB, middle, MSB (matches your sketch: DIP1=DIP LSB)
  GetTaskNumber(uint8_t dip1Pin, uint8_t dip2Pin, uint8_t dip3Pin);

  // Configure pins (call in setup)
  void begin();

  // Read and return task number 0..7. ON = LOW is inverted logic (matches your wiring).
  uint8_t readTask();

private:
  uint8_t dip1Pin;
  uint8_t dip2Pin;
  uint8_t dip3Pin;
};

#endif // GET_TASK_NUMBER_H
