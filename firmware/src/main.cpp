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

extern const uint32_t g_ADigitalPinMap[];

void dumpPin(uint8_t pin) {
  uint32_t p = g_ADigitalPinMap[pin];

  Serial.print("Arduino pin ");
  Serial.print(pin);
  Serial.print(" -> ");

  Serial.print("P");
  Serial.print((p >> 5) & 0x01);   // port (0 or 1)
  Serial.print(".");
  Serial.println(p & 0x1F);        // bit number
}

void findArduinoPin(uint8_t port, uint8_t bit) {
  void* targetPort = (port == 0) ? NRF_P0 : NRF_P1;
  uint32_t targetMask = (1UL << bit);

  for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
    if (digitalPinToPort(pin) == targetPort &&
        digitalPinToBitMask(pin) == targetMask) {

      Serial.print("P");
      Serial.print(port);
      Serial.print(".");
      Serial.print(bit);
      Serial.print(" -> Arduino pin ");
      Serial.println(pin);
      return;
    }
  }

  Serial.print("P");
  Serial.print(port);
  Serial.print(".");
  Serial.print(bit);
  Serial.println(" -> not mapped to any Arduino pin");
}


//////////////////////////////////
//       Initialization         //
//////////////////////////////////
// Hou Dren Repo
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
  // power::init();
  // storage::init();
  // battery::init();
  // bluetooth::init();
//   Serial.printf(
//   "Port: %u, BitMask: 0x%08lX\r\n",
//   digitalPinToPort(PIN_MUX_A0_BIOZ),
//   digitalPinToBitMask(PIN_MUX_A0_BIOZ)
// );
//   findArduinoPin(0, 11);  // P0.11
// findArduinoPin(0, 12);  // P0.12

  


  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  // pinMode(PIN_HEATER_EN1, OUTPUT);
  // pinMode(PIN_HEATER_EN2, OUTPUT);
  digitalWrite(23, HIGH);
  digitalWrite(22, HIGH);
  // digitalWrite(PIN_HEATER_EN1, HIGH);
  // digitalWrite(PIN_HEATER_EN2, HIGH);

  Serial.println("Turning on mux pins");

#ifdef DEBUG_MODE
  // dbgPrintDetailedPinStatus();
  // dbgPrintInterrupts();
  dbgInfo("Done initializing... Off to sleep!");
#endif

  //suspendLoop(); // This code is event driven -- it does not use the main loop
}

void loop() {
  // #ifndef DEBUG_MODE
  //   if (Serial.available()) {
  //     String cmd = Serial.readStringUntil('\n');
  //     cmd.trim();  // Remove any whitespace or newline
  
  //     if (cmd == "reset") {
  //       Serial.println("Resetting...");
  //       delay(100);  // Ensure the message is sent before reset
  //       NVIC_SystemReset();  // Software reset
  //     }
  //   }  
    
  //   //----- Send data over BLE if connected -----
    
  //   if (Bluefruit.connected() && isNewSampleReady()) {
  //       uint16_t conn_handle = Bluefruit.Connection(0)->handle(); // first connected central
  //       bluetooth::sendSensorData(conn_handle, AppBuff, APPBUFF_SIZE);
  //   }
  //   #endif
    
    // #ifdef DEBUG_MODE
    // // // testing dummy data to flutter
    // if (Bluefruit.connected() && millis() - lastSendTime > 1000) {
    //   uint16_t conn_handle = Bluefruit.Connection(0)->handle(); // first connected central
    //   bluetooth::sendSensorData(conn_handle, AppBuff, APPBUFF_SIZE);
    //   lastSendTime = millis();
    // } 
    // #endif
}
