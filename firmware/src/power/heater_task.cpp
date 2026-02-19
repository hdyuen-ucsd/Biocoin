#include "power/heater_task.h"

#include "HWConfig/constants.h"
#include "power/power.h"

#include <Arduino.h>

using namespace power;

namespace power {
    static void heaterTask(void* pvParameters);
}

void power::startHeaterTask() {
  xTaskCreate(heaterTask,   // Task function
              "Heater PWM", // Task name
              512,             // Stack size (in words)
              nullptr,         // Task parameters
              0,               // Priority (very low)
              nullptr          // Task handle (optional)
  );
}

void power::heaterTask(void* pvParameters) {
  while (true) {
    // LED Heartbeat ON
    if (nrf_gpio_pin_dir_get(PIN_HEATER_EN2) == NRF_GPIO_PIN_DIR_OUTPUT){
        digitalWrite(PIN_HEATER_EN2, HIGH);
    }
    
    digitalWrite(PIN_HEATER_EN1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(kHeaterOn));

    // LED Heartbeat OFF
    digitalWrite(PIN_HEATER_EN1, LOW);
    digitalWrite(PIN_HEATER_EN2, LOW);
    
    vTaskDelay(pdMS_TO_TICKS(kHeaterOff));
  }
}
