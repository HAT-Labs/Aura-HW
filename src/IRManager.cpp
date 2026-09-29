#include "IRManager.h"

IRManager* IRManager::_instance = nullptr;

IRManager::IRManager(int recvPin, int ledPin) 
  : _recvPin(recvPin), _IRLED(digitalPinToPinName(ledPin)), 
    _sending(false), _messageReceived(false), _readUserTime(0), 
    _pulseWidthUs(_BASE_WIDTH_US), _carrierOnUs(_MAX_CARRIER_ON_US) {
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
  return (pulseDuration - _BASE_WIDTH_US) / _STEP_WIDTH_US;
}

void IRManager::setCarrierDuty(int percent) {
  // Whole microseconds, so the on-time actually sent is known exactly
  int onUs = (percent * _CARRIER_PERIOD_US + 50) / 100;
  _carrierOnUs = constrain(onUs, _MIN_CARRIER_ON_US, _MAX_CARRIER_ON_US);
}

int IRManager::getCarrierOnUs() { return _carrierOnUs; }
int IRManager::getCarrierPeriodUs() { return _CARRIER_PERIOD_US; }

void IRManager::sendID() {
  _sending = true;
  _IRLED.period_us(_CARRIER_PERIOD_US); // 28us period = ~36kHz modulation
  // Start oscillating: the LED is on for _carrierOnUs of every period
  _IRLED.pulsewidth_us(_PWM_INVERTED ? _CARRIER_PERIOD_US - _carrierOnUs : _carrierOnUs);
  _stopPulseTimeout.attach_us(&timeoutWrapper, _pulseWidthUs); // Send the unique user ID for the specified duration
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
        _pulseTimer.reset();
        _pulseTimer.start();
      } else {
        _pulseTimer.stop();
        _readUserTime = _pulseTimer.read_us();
        _messageReceived = true;
      }
    }
  }
}

void IRManager::stopPulse() {
  ledOff();
  _sending = false;
}

void IRManager::ledOff() { _IRLED.write(_PWM_INVERTED ? 1.0f : 0.0f); }

void IRManager::isrWrapper() { if (_instance) _instance->handleInterrupt(); }
void IRManager::timeoutWrapper() { if (_instance) _instance->stopPulse(); }