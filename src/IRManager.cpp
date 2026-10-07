#include "IRManager.h"
#include "IntervalDecoder.h"

IRManager* IRManager::_instance = nullptr;

IRManager::IRManager(int recvPin, int ledPin) 
  : _recvPin(recvPin), _IRLED(digitalPinToPinName(ledPin)), 
    _sending(false), _messageReceived(false), _readUserTime(0), 
    _pulseWidthUs(_DEFAULT_BASE_US), _carrierOnUs(_MAX_CARRIER_ON_US),
    _intervalCode(false), _baseUs(_DEFAULT_BASE_US), _stepUs(_DEFAULT_STEP_US), _markCycles(_DEFAULT_MARK_CYCLES),
    _identity(0), _markUs(0), _gapUs(0), _phase(0),
    _carrierTest(false), _pulseStartUs(0), _rxLow(false), _pulseValid(false),
    _receivedId(-1), _havePrevMark(false), _prevMarkStartUs(0), _prevMarkWidthUs(0),
    _rawHead(0), _rawTail(0), _rawDropped(0) {
  _instance = this;
}

void IRManager::begin() {
  pinMode(_recvPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_recvPin), isrWrapper, CHANGE);
  ledOff(); // Keep OFF initially
}

void IRManager::setIdentity(int assignedID) {
  _identity = assignedID;
  updateFrame();
}

void IRManager::setWidthCode(int baseUs, int stepUs) {
  _intervalCode = false;
  _baseUs = constrain(baseUs, 280, 5000); // 280 us = 10 carrier cycles, the receiver's shortest burst
  _stepUs = constrain(stepUs, 20, 1000);
  _havePrevMark = false;
  updateFrame();
}

void IRManager::setIntervalCode(int markCycles, int stepUs) {
  _intervalCode = true;
  _markCycles = constrain(markCycles, 10, 35); // >= 10 cycles to be detected; short marks suit either AGC type
  _stepUs = constrain(stepUs, 20, 1000);
  _havePrevMark = false; // a mark received under the old code never pairs with one under the new code
  updateFrame();
}

void IRManager::updateFrame() {
  if (_intervalCode) {
    // The leading edges of the two marks are 2 * mark + step * ID apart; the gap is one mark long at ID 0
    _markUs = (unsigned long)_markCycles * _CARRIER_PERIOD_US;
    _gapUs = _markUs + (unsigned long)_stepUs * _identity;
    _pulseWidthUs = 2 * _markUs + _gapUs;
  } else {
    _pulseWidthUs = (unsigned long)_baseUs + (unsigned long)_stepUs * _identity;
  }
}

bool IRManager::isIntervalCode() { return _intervalCode; }
int IRManager::getCodeBase() { return _intervalCode ? _markCycles : _baseUs; }
int IRManager::getCodeStepUs() { return _stepUs; }
int IRManager::getFrameUs() { return (int)_pulseWidthUs; }

int IRManager::readIdentity(int pulseDuration) {
  // Interval code: a single pulse is only a mark; frames are decoded from mark pairs (decodeMark)
  if (_intervalCode) return -1;
  // Width code: round to the nearest ID; each ID accepts its nominal width +/- half a step, because the
  // receiver can shorten or lengthen a pulse by several carrier cycles. Outside IDs 0 .. _MAX_IDS-1: -1.
  long fromFirstWindow = (long)pulseDuration - (long)_baseUs + (long)_stepUs / 2;
  if (fromFirstWindow < 0) return -1;
  long id = fromFirstWindow / (long)_stepUs;
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
  _pulseValid = false; // a pulse already in progress now overlaps this burst: discard it
  pushRawEvent('T', micros(), _pulseWidthUs); // T line: frame start and whole frame length
  startCarrier();
  if (_intervalCode) {
    _phase = 1; // first mark; onTimeout() runs the gap and the second mark
    _stopPulseTimeout.attach_us(&timeoutWrapper, _markUs);
  } else {
    _phase = 0;
    _stopPulseTimeout.attach_us(&timeoutWrapper, _pulseWidthUs); // Send the unique user ID for the specified duration
  }
}

void IRManager::startCarrierTest(uint32_t ms) {
  _carrierTest = true;
  _sending = true;
  _pulseValid = false;
  _phase = 0;
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
int IRManager::getReceivedIdentity() { return _receivedId; }

// Interval code: pair this mark with the previous one. Returns the frame's ID if the two form a valid frame (both
// marks are then used up), else -1 and this mark waits for the next. Same algorithm and constants as the offline
// decoder (tools/e4_analyze.py), so on-node and offline decodes of the same marks agree.
int IRManager::decodeMark(uint32_t startUs, uint32_t widthUs) {
  if (_havePrevMark) {
    int id = IntervalDecoder::decodePair(_prevMarkStartUs, _prevMarkWidthUs, startUs, widthUs, _markCycles, _stepUs);
    if (id >= 0) {
      _havePrevMark = false;
      return id;
    }
  }
  _havePrevMark = true;
  _prevMarkStartUs = startUs;
  _prevMarkWidthUs = widthUs;
  return -1;
}
void IRManager::clearMessageFlag() { _messageReceived = false; }

// Only pulses that never overlap this node's own transmission are recorded. The own LED's light reaches the
// receiver, so a neighbor's pulse that runs into the own burst would only end when the own burst ends, and
// its stretched width would decode as a wrong ID (E4, 2026-10-07). Discarding it turns that into a miss.
// The receiver level is followed on every edge, even while sending, so it is never stale afterwards.
void IRManager::handleInterrupt() {
  bool low = digitalRead(_recvPin) == LOW; // IR receivers pull LOW while they detect a burst
  if (low == _rxLow) return;
  _rxLow = low;
  if (low) {
    _pulseValid = !_sending;               // starts during the own burst (or its own light): never valid
    if (_pulseValid) {
      _pulseStartUs = micros();
      _pulseTimer.reset();
      _pulseTimer.start();
    }
  } else if (_pulseValid) {
    _pulseValid = false;
    _pulseTimer.stop();
    uint32_t widthUs = _pulseTimer.read_us();
    _readUserTime = widthUs;
    // Width code: every pulse is a message, as before. Interval code: only a mark that completes a frame is.
    int id = _intervalCode ? decodeMark(_pulseStartUs, widthUs) : readIdentity((int)widthUs);
    if (!_intervalCode || id >= 0) {
      _receivedId = id;
      _messageReceived = true;
    }
    pushRawEvent('R', _pulseStartUs, widthUs, id);
  }
}

void IRManager::onTimeout() {
  if (_phase == 1) {        // end of the first mark: LED off for the gap
    ledOff();
    _phase = 2;
    _stopPulseTimeout.attach_us(&timeoutWrapper, _gapUs);
  } else if (_phase == 2) { // second mark; the carrier period is still set from the first
    _IRLED.pulsewidth_us(_carrierOnUs);
    _phase = 3;
    _stopPulseTimeout.attach_us(&timeoutWrapper, _markUs);
  } else {                  // end of a burst, of the second mark, or of a carrier test
    ledOff();
    _phase = 0;
    _sending = false;
    _carrierTest = false;
  }
}

void IRManager::pushRawEvent(char kind, uint32_t tUs, uint32_t widthUs, int id) {
  mbed::CriticalSectionLock lock;
  if (_rawHead - _rawTail >= (uint32_t)_RAW_BUFFER_SIZE) {
    _rawDropped++;
    return;
  }
  RawEvent& slot = _rawEvents[_rawHead % _RAW_BUFFER_SIZE];
  slot.kind = kind;
  slot.tUs = tUs;
  slot.widthUs = widthUs;
  slot.id = (int16_t)id;
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
void IRManager::timeoutWrapper() { if (_instance) _instance->onTimeout(); }