#include "bluetooth/gatt.h"
#include "HWConfig/constants.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/transmitdata_task.h"
#include "util/debug_log.h"
#include "sensors/SensorManager.h"
#include "storage/storage.h"

#include <bluefruit.h>

volatile ImpedanceSample latestSample;
const int packetSize = sizeof(latestSample);
float bioZ1[] = {242, 252, 262, 270, 285, 300, 315, 350, 365, 380, 390, 400};
float bioZ2[] = {203.6, 212, 223, 255, 231, 241, 252, 264, 274, 273, 265, 258};
int sampleIndex = 0;
std::vector<uint8_t> muxChannels;
int muxIndex = 0;

namespace bluetooth {
  BLECharacteristic chrStatus(kUUIDChrStatus);
  BLECharacteristic chrDeviceName(kUUIDChrDeviceName);
  BLECharacteristic chrSensorCtrl(kUUIDChrSensorCtrl);
  BLECharacteristic chrSensorData(kUUIDChrSensorData);
  BLECharacteristic chrSensorParams(kUUIDChrSensorParams);
  BLECharacteristic chrPinConfig(kUUIDChrPinConfig);
  BLECharacteristic chrPinCtrl(kUUIDChrPinCtrl);
  
} // namespace bluetooth

using namespace bluetooth;

void bluetooth::initGatt() {
  dbgInfo("Initializing custom BLE services...");

  bleService.begin();

  // Status characteristic
  chrStatus.setProperties(CHR_PROPS_READ);
  chrStatus.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrStatus.setFixedLen(1);
  chrStatus.begin();
  chrStatus.write8(0);

  // Device name characteristic
  chrDeviceName.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  chrDeviceName.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDeviceName.setMaxLen(BLE_GAP_DEVNAME_MAX_LEN);
  chrDeviceName.begin();
  chrDeviceName.write(storage::readDeviceName().c_str());
  chrDeviceName.setWriteCallback(onNameWrite);

  // Sensor control
  chrSensorCtrl.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrSensorCtrl.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrSensorCtrl.setFixedLen(1);
  chrSensorCtrl.begin();
  chrSensorCtrl.setWriteCallback(onSensorControl);

  // Sensor Parameters
  chrSensorParams.setProperties(CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_WRITE);
  chrSensorParams.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrSensorParams.setMaxLen(kMTURequest - kATTHeaderLen);
  chrSensorParams.begin();
  chrSensorParams.setWriteCallback(onSensorParameters);

  // Sensor Data
  chrSensorData.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  chrSensorData.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrSensorData.setMaxLen(kMTURequest - kATTHeaderLen);                        // We set this to the MAX, but we will only send in MTU chunks
  chrSensorData.begin();

  //Pin Configuration
  chrPinConfig.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  chrPinConfig.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrPinConfig.setMaxLen(BLE_GAP_DEVNAME_MAX_LEN);
  chrPinConfig.begin();
  chrPinConfig.setWriteCallback(onConfigPins);

  //Pin Control
  chrPinCtrl.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  chrPinCtrl.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrPinCtrl.setMaxLen(2);
  chrPinCtrl.begin();
  chrPinCtrl.setWriteCallback(onControlPins);

}

template <typename T>
void bluetooth::clearQueue(std::queue<T>& q) {
  while (!q.empty()) {
    q.pop();
  }
}

void bluetooth::onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  String name = String(reinterpret_cast<const char*>(data)).substring(0, len);

  // Limit length to max allowed (with null terminator)
  if (name.length() > BLE_GAP_DEVNAME_MAX_LEN - 1) name = name.substring(0, BLE_GAP_DEVNAME_MAX_LEN - 1);

  dbgInfo("Updating device name = " + name);
  storage::writeDeviceName(name);
}

void bluetooth::onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  dbgInfo("Received EChem Control Command");
  if (sensor::controlCommand(data, len)) clearQueue(TX_queue);
  // dbgInfo("Received BioZ Control Command");
  // uint8_t well = data[0];
  // uint8_t command = data[1];
  // command == 1 ? startMuxChannel(well) : stopMuxChannel(well);
}

void bluetooth::onConfigPins(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  dbgInfo("Received Pin Configuration");
  // int type = data[];
  // int pinNumber = data[];
  // int direction = data[];
  // pinMode(pinNumber, direction);
}

void bluetooth::onControlPins(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  dbgInfo("Received Pin Control");
  int pinNumber = data[0];
  int mode = data[1];
  dbgInfo("Setting pin " + String(pinNumber) + " to mode " + String(mode));
  digitalWrite(pinNumber, mode);
}

// void bluetooth::startMuxChannel(uint8_t channel) {
//     if (std::find(muxChannels.begin(), muxChannels.end(), channel-1) == muxChannels.end()) {
//         muxChannels.push_back(channel-1);
//     }
//     Serial.println("Start Well: " + String(channel));
// }

// void bluetooth::stopMuxChannel(uint8_t channel) {
//     muxChannels.erase(std::remove(muxChannels.begin(), muxChannels.end(), channel-1), muxChannels.end());
//     Serial.println("Stop Well: " + String(channel));
// }

void bluetooth::onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  //Serial.println("Received Sensor Parameters");
  sensor::loadParameters(data, len);
}

void bluetooth::sendSensorData(uint16_t conn_hdl, uint32_t *pData, uint32_t DataCount) {
  #ifndef DEBUG_MODE
  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  
  float impMag = pImp[0].Magnitude;
  uint8_t channel = getMUXChannel();
  latestSample.Impedance = impMag;
  latestSample.Channel = channel;
  uint32_t timestamp = millis();
  uint8_t buffer[packetSize];
  buffer[0] = channel;
  memcpy(&buffer[1], &impMag, sizeof(impMag));
  memcpy(&buffer[1 + sizeof(impMag)], &timestamp, sizeof(timestamp));
  bool isSelectedChannel = std::find(muxChannels.begin(), muxChannels.end(), channel) != muxChannels.end();
  if (isNewSampleReady() && isSelectedChannel) {
      //Serial.println("Channel: " + String(channel) + " Sending Sensor Data: " + String(impMag) + " at timestamp: " + String(timestamp));
      chrSensorData.notify(conn_hdl, buffer, sizeof(buffer));
      clearNewSampleFlag();
  }
  #endif

  // for testing with flutter
  #ifdef DEBUG_MODE
  //Serial.println("Sending Dummy Sensor Data");
  float impMag = sampleIndex % 2 == 0 ? bioZ1[sampleIndex / 2] : bioZ2[sampleIndex / 2];

  uint8_t channel = sampleIndex % 2; // Alternate between channel 0 and 1
  uint32_t timestamp = millis();
  uint8_t buffer[packetSize];
  buffer[0] = channel;
  memcpy(&buffer[1], &impMag, sizeof(impMag));
  memcpy(&buffer[1 + sizeof(impMag)], &timestamp, sizeof(timestamp));
  bool isSelectedChannel = std::find(muxChannels.begin(), muxChannels.end(), channel) != muxChannels.end();
  Serial.println("Channel: " + String(channel) + " is selected: " + String(isSelectedChannel));
  for (int i = 0; i < muxChannels.size(); i++) {
    Serial.println("Selected Channel: " + String(muxChannels[i]));
  }
  if (isSelectedChannel) {
      chrSensorData.notify(conn_hdl, buffer, sizeof(buffer));
  }
  sampleIndex = (sampleIndex + 1) % 24;
  #endif
}
