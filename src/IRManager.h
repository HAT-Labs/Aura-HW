#ifndef IR_MANAGER_H
#define IR_MANAGER_H

#include <Arduino.h>
#include "mbed.h"

class IRManager {
public:
  // Raw event for the E0 measurement log: 'R' = received pulse (start time, width),
  // 'T' = own transmission (start time, burst length). Times are micros().
  struct RawEvent {
    char kind;
    uint32_t tUs;
    uint32_t widthUs;
    int16_t id; // 'R': ID decoded by the node (width code: this pulse; interval code: the frame this pulse
                // completes, logged on its second mark); -1 = none
  };

  IRManager(int recvPin, int ledPin);
  void begin();
  void sendID();
  bool hasNewMessage();
  int getReceivedTime();
  int getReceivedIdentity(); // ID of the latest message, decoded in the receiver ISR for either code; -1 = none
  void clearMessageFlag();
  void setIdentity(int assignedID);
  int readIdentity(int pulseDuration); // width code: rounding decoder for one pulse width
  void setCarrierDuty(int percent); // LED on-time per carrier period, rounded to whole us (7-50 %)
  int getCarrierOnUs();
  int getCarrierPeriodUs();
  void startCarrierTest(uint32_t ms); // continuous carrier at the current duty, then off; beacons pause
  void setWidthCode(int baseUs, int stepUs);        // width code: one burst of base + step * ID
  void setIntervalCode(int markCycles, int stepUs); // interval code: mark, gap, mark; ID in the leading-edge spacing
  bool isIntervalCode();
  int getCodeBase();   // width code: base in us; interval code: mark length in carrier cycles
  int getCodeStepUs();
  int getFrameUs();    // on-air length of this node's ID (one burst, or mark + gap + mark)
  bool popRawEvent(RawEvent& event);
  uint32_t takeDroppedEvents();       // events lost because the buffer was full since the last call

private:
  int _recvPin;
  mbed::PwmOut _IRLED;
  mbed::Timer _pulseTimer;
  mbed::Timeout _stopPulseTimeout;

  volatile bool _sending;
  volatile bool _messageReceived;
  volatile int _readUserTime;
  volatile unsigned long _pulseWidthUs; // whole frame: the burst (width code) or mark + gap + mark (interval code)
  int _carrierOnUs;
  bool _intervalCode;
  int _baseUs, _stepUs, _markCycles, _identity;
  unsigned long _markUs, _gapUs;
  volatile int _phase; // interval frame in progress: 1 = first mark, 2 = gap, 3 = second mark; 0 otherwise
  volatile bool _carrierTest;
  volatile uint32_t _pulseStartUs;
  volatile bool _rxLow;      // receiver output as last seen by the ISR (LOW = burst detected), tracked even while sending
  volatile bool _pulseValid; // the pulse in progress has not overlapped this node's own transmission
  volatile int _receivedId;  // latest decoded ID (getReceivedIdentity)
  // Interval code: the previous mark, waiting for the mark that completes its frame
  volatile bool _havePrevMark;
  volatile uint32_t _prevMarkStartUs, _prevMarkWidthUs;

  // Written from the receiver ISR and from sendID(), read by loop(); guarded by a critical section
  static const int _RAW_BUFFER_SIZE = 64;
  RawEvent _rawEvents[_RAW_BUFFER_SIZE];
  volatile uint32_t _rawHead;
  volatile uint32_t _rawTail;
  volatile uint32_t _rawDropped;

  static const int _DEFAULT_BASE_US = 1000;  // the original code: ID n = 1000 + 400 * n us
  static const int _DEFAULT_STEP_US = 400;
  static const int _DEFAULT_MARK_CYCLES = 12;
  static const int _MAX_IDS = 16; // irLookedBitmask is 16 bits

  // Carrier: 28 us period = ~36 kHz. The LED on-time is set in whole us, 2-14 us (7-50 %);
  // Vishay's receivers accept 5-50 % carrier duty.
  static const int _CARRIER_PERIOD_US = 28;
  static const int _MIN_CARRIER_ON_US = 2;
  static const int _MAX_CARRIER_ON_US = 14;

  // PwmOut on this core: pulsewidth_us(n) drives D8 HIGH for n us per period, and D8 HIGH turns the low-side
  // MOSFET and LED on (scope, 2026-09-30). The exception is 100 %: write(1.0f) holds D8 LOW, so it is the
  // LED-off state (~0 V across the 20 ohm resistor at idle, 2026-09-29).

  static IRManager* _instance;
  static void isrWrapper();
  static void timeoutWrapper();
  
  void handleInterrupt();
  void onTimeout();
  void updateFrame();
  void ledOff();
  void startCarrier();
  int decodeMark(uint32_t startUs, uint32_t widthUs);
  void pushRawEvent(char kind, uint32_t tUs, uint32_t widthUs, int id = -1);
};
#endif