#include "power/heater_task.h"
#include "HWConfig/constants.h"
#include "power/power.h"
#include "util/debug_log.h"

#include <Arduino.h>


namespace power {
    static void heaterTask1(void* pvParameters);
    static void heaterTask2(void* pvParameters);
    volatile bool heatingSuspended = false;
    volatile bool heaterOff = false;
    static volatile uint32_t heaterOffDelayMs = 50;
    static volatile uint32_t resumeTimeMs = 0;
    static const uint32_t kResumeDelayMs = 50;
    static volatile uint8_t kHeaterDutyCycle1 = 50; // Default duty cycle percentage (0-100)  
    static volatile uint8_t kHeaterDutyCycle2 = 50; // Default duty cycle percentage (0-100)  
    static uint32_t kHeaterOn1;
    static uint32_t kHeaterOff1;
    static uint32_t kHeaterOn2;
    static uint32_t kHeaterOff2;

    void startHeaterTask() {
      xTaskCreate(heaterTask1,   // Task function
                  "Heater PWM1", // Task name
                  512,             // Stack size (in words)
                  nullptr,         // Task parameters
                  0,               // Priority (very low)
                  nullptr          // Task handle (optional)
      );
      xTaskCreate(heaterTask2,   // Task function
                  "Heater PWM2", // Task name
                  512,             // Stack size (in words)
                  nullptr,         // Task parameters
                  0,               // Priority (very low)
                  nullptr          // Task handle (optional)
      );
    }

    void heaterTask1(void* pvParameters) {
      while (true) {
        // LED Heartbeat ON
        if (heatingSuspended){
          //dbgInfo("Heating suspended");
          digitalWrite(PIN_HEATER_EN1, LOW);
          vTaskDelay(pdMS_TO_TICKS(heaterOffDelayMs));
          heaterOff = true;
          continue;
        }
        if ((millis() - resumeTimeMs) < kResumeDelayMs)
        {
            digitalWrite(PIN_HEATER_EN1, LOW);
            heaterOff = true;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        heaterOff = false;
        kHeaterOn1 = (kHeaterDutyCycle1 * kHeaterCyclePeriod) / 100;
        kHeaterOff1 = kHeaterCyclePeriod - kHeaterOn1;

        if (kHeaterOn1 == 0) {
            digitalWrite(PIN_HEATER_EN1, LOW);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }

        if (kHeaterOff1 == 0) {
            digitalWrite(PIN_HEATER_EN1, HIGH);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }
        //dbgInfo("Heater on");
        digitalWrite(PIN_HEATER_EN1, HIGH);
        vTaskDelay(pdMS_TO_TICKS(kHeaterOn1));
        //dbgInfo("Heater off");
        // LED Heartbeat OFF
        digitalWrite(PIN_HEATER_EN1, LOW);
        
        vTaskDelay(pdMS_TO_TICKS(kHeaterOff1));
      }
    }

    void heaterTask2(void* pvParameters) {
      while (true) {
        // LED Heartbeat ON
        if (heatingSuspended){
          //dbgInfo("Heating suspended");
          digitalWrite(PIN_HEATER_EN2, LOW);
          vTaskDelay(pdMS_TO_TICKS(heaterOffDelayMs));          heaterOff = true;
          continue;
        }
        if ((millis() - resumeTimeMs) < kResumeDelayMs)
        {
            digitalWrite(PIN_HEATER_EN2, LOW);
            heaterOff = true;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        heaterOff = false;
        kHeaterOn2 = (kHeaterDutyCycle2 * kHeaterCyclePeriod) / 100;
        kHeaterOff2 = kHeaterCyclePeriod - kHeaterOn2;

        if (kHeaterOn2 == 0) {
            digitalWrite(PIN_HEATER_EN2, LOW);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }

        if (kHeaterOff2 == 0) {
            digitalWrite(PIN_HEATER_EN2, HIGH);
            vTaskDelay(pdMS_TO_TICKS(kHeaterCyclePeriod));
            continue;
        }
        //dbgInfo("Heater on");
        digitalWrite(PIN_HEATER_EN2, HIGH);
        vTaskDelay(pdMS_TO_TICKS(kHeaterOn2));
        //dbgInfo("Heater off");
        // LED Heartbeat OFF
        digitalWrite(PIN_HEATER_EN2, LOW);
        
        vTaskDelay(pdMS_TO_TICKS(kHeaterOff2));
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

    void setHeaterDutyCycle(uint8_t dutyCycle, uint8_t heaterChannel)
    {
      if (dutyCycle > 100) dutyCycle = 100;

      if (heaterChannel == 1) {
        if (kHeaterDutyCycle1 != dutyCycle)
        {
          kHeaterDutyCycle1 = dutyCycle;
          dbgInfo("Heater 1 duty cycle set to " + String(dutyCycle) + "%");
        }
      } else if (heaterChannel == 2) {
        if (kHeaterDutyCycle2 != dutyCycle)
        {
          kHeaterDutyCycle2 = dutyCycle;
          dbgInfo("Heater 2 duty cycle set to " + String(dutyCycle) + "%");
        }
      }
    }
}
