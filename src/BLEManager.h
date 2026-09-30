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
  bool isMagRequested();
  bool isGyroRequested();
  // True once per config write that carried the optional third byte (IR carrier duty, percent)
  bool takeCarrierDutyUpdate(int& percent);
  // True once per config write that carried the optional fourth byte (beacon jitter, ms)
  bool takeBeaconJitterUpdate(int& ms);
  // This chip's 64-bit factory device ID as 16 hex characters; tells nodes apart in logs
  static String nodeUid();

  // Sends the entire consolidated struct over the air
  void sendPacket(const SensorPacket& packet);

private:
  SystemState _currentState;
  int _assignedID;
  byte _requestedModalities;
  int _carrierDutyPercent;
  bool _carrierDutyUpdated;
  int _beaconJitterMs;
  bool _beaconJitterUpdated;

  BLEService _sensorService;
  BLECharacteristic _configChar;
  BLECharacteristic _dataChar; // Adjusted for structural data payloads
  BLECharacteristic _uidChar;

  void handleConfigWrite();
};

#endif