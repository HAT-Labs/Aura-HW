#include "IRManager.h"

IRManager* IRManager::_instance = nullptr;

IRManager::IRManager(int recvPin, int ledPin) 
  : _recvPin(recvPin), _IRLED(digitalPinToPinName(ledPin)), 
    _sending(false), _messageReceived(false), _readUserTime(0), 
    _pulseWidthUs(_BASE_WIDTH_US) {
  _instance = this;
}

void IRManager::begin() {
  pinMode(_recvPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_recvPin), isrWrapper, CHANGE);
  _IRLED.write(1.0f); // Keep OFF initially (Low-Side MOSFET optimization)
}

void IRManager::setIdentity(int assignedID) {
  // identifiedUser = (pulseDuration - 900) / 200;
  _pulseWidthUs = _BASE_WIDTH_US + (unsigned long)(assignedID * _STEP_WIDTH_US);
}

void IRManager::sendID() {
  _sending = true;
  _IRLED.period_us(26); // 26us period = ~38kHz modulation
  _IRLED.write(0.5f);   // Start oscillating
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
  _IRLED.write(1.0f); // Pull HIGH to turn OFF low-side MOSFET layout
  _sending = false;
}

void IRManager::isrWrapper() { if (_instance) _instance->handleInterrupt(); }
void IRManager::timeoutWrapper() { if (_instance) _instance->stopPulse(); }