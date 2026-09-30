#include "IRManager.h"

IRManager* IRManager::_instance = nullptr;

IRManager::IRManager(int recvPin, int ledPin) 
  : _recvPin(recvPin), _IRLED(digitalPinToPinName(ledPin)), 
    _sending(false), _messageReceived(false), _readUserTime(0), 
    _pulseWidthUs(_BASE_WIDTH_US), _carrierOnUs(_MAX_CARRIER_ON_US),
    _carrierTest(false), _pulseStartUs(0), _rawHead(0), _rawTail(0), _rawDropped(0) {
  _instance = this;
}

void IRManager::begin() {
  pinMode(_recvPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_recvPin), isrWrapper, CHANGE);
  ledOff(); // Keep OFF initially
}

void IRManager::setIdentity(int assignedID) {
  // identifiedUser = (pulseDuration - 900) / 200;
  _pulseWidthUs = _BASE_WIDTH_US + (unsigned long)(assignedID * _STEP_WIDTH_US);
}

int IRManager::readIdentity(int pulseDuration) {
  // Round to the nearest ID: each ID accepts its nominal width +/- half a step, because the receiver
  // can shorten or lengthen a pulse by several carrier cycles. Anything outside the windows of
  // IDs 0 .. _MAX_IDS-1 is noise and returns -1. Signed math: the constants are unsigned.
  long fromFirstWindow = (long)pulseDuration - (long)_BASE_WIDTH_US + (long)_STEP_WIDTH_US / 2;
  if (fromFirstWindow < 0) return -1;
  long id = fromFirstWindow / (long)_STEP_WIDTH_US;
  return id < _MAX_IDS ? (int)id : -1;
}

void IRManager::setCarrierDuty(int percent) {
  // Whole microseconds, so the on-time actually sent is known exactly
  int onUs = (percent * _CARRIER_PERIOD_US + 50) / 100;
  _carrierOnUs = constrain(onUs, _MIN_CARRIER_ON_US, _MAX_CARRIER_ON_US);
}

int IRManager::getCarrierOnUs() { return _carrierOnUs; }
int IRManager::getCarrierPeriodUs() { return _CARRIER_PERIOD_US; }

void IRManager::sendID() {
  if (_carrierTest) return; // a carrier test owns the LED until it ends
  _sending = true;
  pushRawEvent('T', micros(), _pulseWidthUs);
  startCarrier();
  _stopPulseTimeout.attach_us(&timeoutWrapper, _pulseWidthUs); // Send the unique user ID for the specified duration
}

void IRManager::startCarrierTest(uint32_t ms) {
  _carrierTest = true;
  _sending = true;
  startCarrier();
  _stopPulseTimeout.attach_us(&timeoutWrapper, (us_timestamp_t)ms * 1000);
}

void IRManager::startCarrier() {
  _IRLED.period_us(_CARRIER_PERIOD_US); // 28us period = ~36kHz modulation
  // Start oscillating: the LED is on for _carrierOnUs of every period
  _IRLED.pulsewidth_us(_carrierOnUs);
}

bool IRManager::hasNewMessage() { return _messageReceived; }
int IRManager::getReceivedTime() { return _readUserTime; }
void IRManager::clearMessageFlag() { _messageReceived = false; }

void IRManager::handleInterrupt() {
  if (!_sending) {
    static bool lastState = HIGH;
    bool currentState = digitalRead(_recvPin);
    if (currentState != lastState) {
      lastState = currentState;
      if (currentState == LOW) { // IR Receivers usually pull LOW when detecting light
        _pulseStartUs = micros();
        _pulseTimer.reset();
        _pulseTimer.start();
      } else {
        _pulseTimer.stop();
        _readUserTime = _pulseTimer.read_us();
        _messageReceived = true;
        pushRawEvent('R', _pulseStartUs, _readUserTime);
      }
    }
  }
}

void IRManager::stopPulse() {
  ledOff();
  _sending = false;
  _carrierTest = false;
}

void IRManager::pushRawEvent(char kind, uint32_t tUs, uint32_t widthUs) {
  mbed::CriticalSectionLock lock;
  if (_rawHead - _rawTail >= (uint32_t)_RAW_BUFFER_SIZE) {
    _rawDropped++;
    return;
  }
  RawEvent& slot = _rawEvents[_rawHead % _RAW_BUFFER_SIZE];
  slot.kind = kind;
  slot.tUs = tUs;
  slot.widthUs = widthUs;
  _rawHead++;
}

bool IRManager::popRawEvent(RawEvent& event) {
  mbed::CriticalSectionLock lock;
  if (_rawTail == _rawHead) return false;
  event = _rawEvents[_rawTail % _RAW_BUFFER_SIZE];
  _rawTail++;
  return true;
}

uint32_t IRManager::takeDroppedEvents() {
  mbed::CriticalSectionLock lock;
  uint32_t dropped = _rawDropped;
  _rawDropped = 0;
  return dropped;
}

void IRManager::ledOff() { _IRLED.write(1.0f); } // holds D8 LOW on this core (see IRManager.h)

void IRManager::isrWrapper() { if (_instance) _instance->handleInterrupt(); }
void IRManager::timeoutWrapper() { if (_instance) _instance->stopPulse(); }