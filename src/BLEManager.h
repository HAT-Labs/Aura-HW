#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "DataPacket.h"

enum SystemState {
  STATE_DISCONNECTED,
  STATE_WAITING_FOR_CONFIG,
  STATE_STREAMING
};

class BLEManager {
public:
  BLEManager();
  bool begin();
  void update();
  
  SystemState getState();
  int getAssignedID();
  bool isIRRequested();
  bool isIMURequested();

  // Sends the entire consolidated struct over the air
  void sendPacket(const SensorPacket& packet);

private:
  SystemState _currentState;
  int _assignedID;
  byte _requestedModalities;

  BLEService _sensorService;
  BLECharacteristic _configChar;
  BLECharacteristic _dataChar; // Adjusted for structural data payloads

  void handleConfigWrite();
};

#endif