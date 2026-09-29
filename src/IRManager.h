#ifndef IR_MANAGER_H
#define IR_MANAGER_H

#include <Arduino.h>
#include "mbed.h"

class IRManager {
public:
  IRManager(int recvPin, int ledPin);
  void begin();
  void sendID();
  bool hasNewMessage();
  int getReceivedTime();
  void clearMessageFlag();
  void setIdentity(int assignedID);
  int readIdentity(int pulseDuration);
  void setCarrierDuty(int percent); // LED on-time per carrier period, rounded to whole us (7-50 %)
  int getCarrierOnUs();
  int getCarrierPeriodUs();

private:
  int _recvPin;
  mbed::PwmOut _IRLED;
  mbed::Timer _pulseTimer;
  mbed::Timeout _stopPulseTimeout;

  volatile bool _sending;
  volatile bool _messageReceived;
  volatile int _readUserTime;
  volatile unsigned long _pulseWidthUs;
  int _carrierOnUs;

  static const unsigned long _BASE_WIDTH_US = 1000;
  static const unsigned long _STEP_WIDTH_US = 400;
  static const int _MAX_IDS = 16; // irLookedBitmask is 16 bits

  // Carrier: 28 us period = ~36 kHz. The LED on-time is set in whole us, 2-14 us (7-50 %);
  // Vishay's receivers accept 5-50 % carrier duty.
  static const int _CARRIER_PERIOD_US = 28;
  static const int _MIN_CARRIER_ON_US = 2;
  static const int _MAX_CARRIER_ON_US = 14;

  // On this core the PwmOut output is inverted relative to mbed's docs: write(1.0f) holds D8 LOW, so the
  // low-side MOSFET and LED are off (checked 2026-09-29: ~0 V across the 20 ohm resistor at idle).
  // Scope check still pending that pulse widths invert too: DUTY 25 must light the LED 7 of every 28 us.
  static const bool _PWM_INVERTED = true;

  static IRManager* _instance;
  static void isrWrapper();
  static void timeoutWrapper();
  
  void handleInterrupt();
  void stopPulse();
  void ledOff();
};
#endif