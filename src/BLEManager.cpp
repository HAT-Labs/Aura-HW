#include "BLEManager.h"

#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CONFIG_CHAR_UUID    "19B10001-E8F2-537E-4F6C-D104768A1214"
#define DATA_CHAR_UUID      "19B10002-E8F2-537E-4F6C-D104768A1214"
#define UID_CHAR_UUID       "19B10003-E8F2-537E-4F6C-D104768A1214"

BLEManager::BLEManager() : 
  _currentState(STATE_DISCONNECTED), _assignedID(-1), _requestedModalities(0),
  _carrierDutyPercent(50), _carrierDutyUpdated(false), _beaconJitterMs(20), _beaconJitterUpdated(false),
  _sensorService(SERVICE_UUID),
  // {ID, modality mask}, optionally followed by IR carrier duty % and beacon jitter ms
  _configChar(CONFIG_CHAR_UUID, BLERead | BLEWrite, 4),
  // Allocate characteristic size dynamically using our struct dimension
  _dataChar(DATA_CHAR_UUID, BLERead | BLENotify, sizeof(SensorPacket)),
  _uidChar(UID_CHAR_UUID, BLERead, 16)
{}

String BLEManager::nodeUid() {
  char uid[17];
  snprintf(uid, sizeof(uid), "%08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
  return String(uid);
}

bool BLEManager::begin() {
  if (!BLE.begin()) return false;

  BLE.setLocalName("SocialMonitorNode");
  BLE.setAdvertisedService(_sensorService);

  // Data is getting transmitted at 10 Hz
  BLE.setConnectionInterval(0x50, 0x60); // 100ms to 120ms

  _sensorService.addCharacteristic(_configChar);
  _sensorService.addCharacteristic(_dataChar);
  _sensorService.addCharacteristic(_uidChar);
  BLE.addService(_sensorService);
  _uidChar.writeValue(nodeUid().c_str());
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
  int length = _configChar.valueLength();
  if (length >= 2 && length <= 4) {
    _assignedID = payload[0];
    _requestedModalities = payload[1];
    if (length >= 3) {
      _carrierDutyPercent = payload[2];
      _carrierDutyUpdated = true;
    }
    if (length == 4) {
      _beaconJitterMs = payload[3];
      _beaconJitterUpdated = true;
    }
    _currentState = STATE_STREAMING;
  }
}

bool BLEManager::takeCarrierDutyUpdate(int& percent) {
  if (!_carrierDutyUpdated) return false;
  _carrierDutyUpdated = false;
  percent = _carrierDutyPercent;
  return true;
}

bool BLEManager::takeBeaconJitterUpdate(int& ms) {
  if (!_beaconJitterUpdated) return false;
  _beaconJitterUpdated = false;
  ms = _beaconJitterMs;
  return true;
}

SystemState BLEManager::getState() { return _currentState; }
int BLEManager::getAssignedID() { return _assignedID; }
bool BLEManager::isIRRequested() { return (_requestedModalities & 0x01); }
bool BLEManager::isIMURequested() { return (_requestedModalities & 0x02); }
bool BLEManager::isMagRequested() { return (_requestedModalities & 0x04); }
bool BLEManager::isGyroRequested() { return (_requestedModalities & 0x08); }

void BLEManager::sendPacket(const SensorPacket& packet) {
  if (_currentState != STATE_STREAMING) return;
  
  // Hand over the memory address of the struct casted to a byte array
  _dataChar.writeValue((const uint8_t*)&packet, sizeof(SensorPacket));
}