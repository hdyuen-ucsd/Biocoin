#pragma once

namespace power {
  void startHeaterTask();
  void suspendHeating();
  void resumeHeating();
  bool isHeaterOff();
  extern volatile bool heatingSuspended;
  extern volatile bool heaterOff;
  
} // namespace power
