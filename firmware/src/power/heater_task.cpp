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
    static const uint32_t kResumeDelayMs = 1000;
    static volatile uint8_t kHeaterDutyCycle = 50; // Default duty cycle percentage (0-100)  
    static uint32_t kHeaterOn;
    static uint32_t kHeaterOff;

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
        kHeaterOn = (kHeaterDutyCycle * kHeaterCyclePeriod) / 100;
        kHeaterOff = kHeaterCyclePeriod - kHeaterOn;

        if (kHeaterOn == 0) {
            digitalWrite(PIN_HEATER_EN1, LOW);
            digitalWrite(PIN_HEATER_EN2, LOW);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }

        if (kHeaterOff == 0) {
            digitalWrite(PIN_HEATER_EN1, HIGH);
            digitalWrite(PIN_HEATER_EN2, LOW);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }
        //dbgInfo("Heater on");
        digitalWrite(PIN_HEATER_EN2, LOW);
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

    void setHeaterDutyCycle(uint8_t dutyCycle)
    {
      if (dutyCycle > 100) dutyCycle = 100;

      if (kHeaterDutyCycle != dutyCycle)
      {
          kHeaterDutyCycle = dutyCycle;
          dbgInfo("Heater duty cycle set to " + String(dutyCycle) + "%");
      }
    }
}