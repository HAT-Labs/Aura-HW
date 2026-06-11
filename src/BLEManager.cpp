#include "BLEManager.h"

#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CONFIG_CHAR_UUID    "19B10001-E8F2-537E-4F6C-D104768A1214"
#define DATA_CHAR_UUID      "19B10002-E8F2-537E-4F6C-D104768A1214"

BLEManager::BLEManager() : 
  _currentState(STATE_DISCONNECTED), _assignedID(-1), _requestedModalities(0),
  _sensorService(SERVICE_UUID),
  _configChar(CONFIG_CHAR_UUID, BLERead | BLEWrite, 2),
  // Allocate characteristic size dynamically using our struct dimension
  _dataChar(DATA_CHAR_UUID, BLERead | BLENotify, sizeof(SensorPacket)) 
{}

bool BLEManager::begin() {
  if (!BLE.begin()) return false;

  BLE.setLocalName("SocialMonitorNode");
  BLE.setAdvertisedService(_sensorService);
  _sensorService.addCharacteristic(_configChar);
  _sensorService.addCharacteristic(_dataChar);
  BLE.addService(_sensorService);
  BLE.advertise();
  return true;
}

void BLEManager::update() {
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (_currentState == STATE_DISCONNECTED) _currentState = STATE_WAITING_FOR_CONFIG;
    if (_configChar.written()) handleConfigWrite();
  } else {
    _currentState = STATE_DISCONNECTED;
  }
}

void BLEManager::handleConfigWrite() {
  const byte* payload = _configChar.value();
  if (_configChar.valueLength() == 2) {
    _assignedID = payload[0];
    _requestedModalities = payload[1];
    _currentState = STATE_STREAMING;
  }
}

SystemState BLEManager::getState() { return _currentState; }
bool BLEManager::isIRRequested() { return (_requestedModalities & 0x01); }
bool BLEManager::isIMURequested() { return (_requestedModalities & 0x02); }

void BLEManager::sendPacket(const SensorPacket& packet) {
  if (_currentState != STATE_STREAMING) return;
  
  // Hand over the memory address of the struct casted to a byte array
  _dataChar.writeValue((const uint8_t*)&packet, sizeof(SensorPacket));
}