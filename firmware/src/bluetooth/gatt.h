#pragma once

#include "bluefruit.h"
#include <queue>
#include <vector>
#include <cstdint>
#include <algorithm>

namespace bluetooth {

  extern BLECharacteristic chrSensorData;
  extern BLECharacteristic chrStatus;


  void initGatt();
  void startMuxChannel(uint8_t channel);
  void stopMuxChannel(uint8_t channel);

  template <typename T>
  void clearQueue(std::queue<T>& q);
  void updateStatus();

  // Callbacks
  void onDataNotifications(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
  void onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);
  void onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  void onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  void sendSensorData(uint16_t conn_hdl, uint32_t *pData, uint32_t DataCount);
  void onConfigPins(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);
  void onControlPins(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);

} // namespace bluetooth


