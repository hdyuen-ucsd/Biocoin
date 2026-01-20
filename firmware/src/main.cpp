//////////////////////////////////
//            Headers           //
//////////////////////////////////
#include "HWConfig/constants.h"
#include "battery/battery.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/gatt.h"
#include "power/power.h"
#include "sensors/SensorManager.h"
#include "storage/storage.h"
#include "util/debug_log.h"
#include "util/util.h"

#include <Arduino.h>
uint32_t AppBuff[APPBUFF_SIZE];
unsigned long lastSendTime = 0;

//////////////////////////////////
//       Initialization         //
//////////////////////////////////
void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200); // The baudrate here does not matter when using USB CDC
  while (!Serial)
    delay(10); // Wait for a serial connection if we are debugging

  delay(1000); // Wait for the serial monitor to start (important)

  dbgInfo(kWelcomeMessage);
  dbgInfo("Version info:");
  dbgPrintVersion();
  dbgPrintResetReason();
#endif

  // Initialize the subsystems
  power::init();
  storage::init();
  battery::init();
  bluetooth::init();
  sensor::init();

#ifdef DEBUG_MODE
  // dbgPrintDetailedPinStatus();
  // dbgPrintInterrupts();
  dbgInfo("Done initializing... Off to sleep!");
#endif

  //suspendLoop(); // This code is event driven -- it does not use the main loop
}

void loop() {
  #ifndef DEBUG_MODE
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();  // Remove any whitespace or newline
  
      if (cmd == "reset") {
        Serial.println("Resetting...");
        delay(100);  // Ensure the message is sent before reset
        NVIC_SystemReset();  // Software reset
      }
    }  
    
    //----- Send data over BLE if connected -----
    
    if (Bluefruit.connected() && isNewSampleReady()) {
        uint16_t conn_handle = Bluefruit.Connection(0)->handle(); // first connected central
        bluetooth::sendSensorData(conn_handle, AppBuff, APPBUFF_SIZE);
    }
    #endif
    
    #ifdef DEBUG_MODE
    // // testing dummy data to flutter
    if (Bluefruit.connected() && millis() - lastSendTime > 1000) {
      uint16_t conn_handle = Bluefruit.Connection(0)->handle(); // first connected central
      bluetooth::sendSensorData(conn_handle, AppBuff, APPBUFF_SIZE);
      lastSendTime = millis();
    } 
    #endif
}
