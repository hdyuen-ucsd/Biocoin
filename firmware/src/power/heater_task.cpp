#include "power/heater_task.h"
#include "HWConfig/constants.h"
#include "power/power.h"
#include "util/debug_log.h"

#include <Arduino.h>

namespace power {
    static void heaterTask(void* pvParameters);
    volatile bool heatingSuspended = false;
    volatile bool heaterOff = false;
    static volatile uint32_t resumeTimeMs = 0;
    static const uint32_t kResumeDelayMs = 100;   // 3–10ms usually enough

    void startHeaterTask() {
      xTaskCreate(heaterTask,   // Task function
                  "Heater PWM", // Task name
                  512,             // Stack size (in words)
                  nullptr,         // Task parameters
                  0,               // Priority (very low)
                  nullptr          // Task handle (optional)
      );
    }

    void heaterTask(void* pvParameters) {
      while (true) {
        // LED Heartbeat ON
        if (heatingSuspended){
          //dbgInfo("Heating suspended");
          digitalWrite(PIN_HEATER_EN1, LOW);
          digitalWrite(PIN_HEATER_EN2, LOW);
          vTaskDelay(pdMS_TO_TICKS(1500));
          heaterOff = true;
          continue;
        }
        if ((millis() - resumeTimeMs) < kResumeDelayMs)
        {
            digitalWrite(PIN_HEATER_EN1, LOW);
            digitalWrite(PIN_HEATER_EN2, LOW);
            heaterOff = true;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        heaterOff = false;
        //dbgInfo("Heater on");
        digitalWrite(PIN_HEATER_EN2, HIGH);
        digitalWrite(PIN_HEATER_EN1, HIGH);
        vTaskDelay(pdMS_TO_TICKS(kHeaterOn));
        //dbgInfo("Heater off");
        // LED Heartbeat OFF
        digitalWrite(PIN_HEATER_EN1, LOW);
        digitalWrite(PIN_HEATER_EN2, LOW);
        
        vTaskDelay(pdMS_TO_TICKS(kHeaterOff));
      }
    }

    void suspendHeating()
    {
      heatingSuspended = true;
    }

    void resumeHeating()
    {
      resumeTimeMs = millis();
      heatingSuspended = false;
    }

    bool isHeaterOff()
    {
      return heaterOff;
    }
}