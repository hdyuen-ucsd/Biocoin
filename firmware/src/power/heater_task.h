#pragma once
#include <stdint.h>

namespace power {
  void startHeaterTask();
  void suspendHeating();
  void resumeHeating();
  bool isHeaterOff();
  void setHeaterDutyCycle(uint8_t dutyCycle);
  extern volatile bool heatingSuspended;
  extern volatile bool heaterOff;
  
} // namespace power
