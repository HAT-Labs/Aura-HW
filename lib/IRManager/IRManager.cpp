#include "IRManager.h"

// Initialize the static instance pointer to nullptr
IRManager* IRManager::_instance = nullptr;

// Constructor to initialize pins and timings
IRManager::IRManager(int recvPin, int ledPin, int outPin, unsigned long uniqueTime)
    : _recvPin(recvPin), _ledPin(ledPin), _outPin(outPin), _uniqueTime(uniqueTime),
      _IRLED(digitalPinToPinName(ledPin)), _sending(false), _receiving(false), _readUserTime(0), _outState(LOW) {
    // Set the static instance pointer to this instance
    _instance = this;
}

void IRManager::begin() {
    pinMode(_outPin, OUTPUT);
    digitalWrite(_outPin, _outState);
    pinMode(_recvPin, INPUT);

    // Attach the static wrapper, not the class method directly, because ISRs can't be class methods
    attachInterrupt(digitalPinToPinName(_recvPin), isrWrapper, CHANGE);
}

void IRManager::sendID() {
    _sending = true;
    _IRLED.period_us(26); // 38kHz period
    _IRLED.write(0.5f); // 50% duty cycle
    _stopPulseTimeout.attach_us(&timeoutWrapper, _uniqueTime);
}

bool IRManager::hasMessage() {
    return _receiving;
}

int IRManager::getReceivedTime() {
    return _readUserTime;
}

void IRManager::acknowledgeMessage() {
    _receiving = false;
}

// --- Internal Logic ---

void IRManager::handleInterrupt() {
    
}