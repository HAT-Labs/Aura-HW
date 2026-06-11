#ifndef IR_MANAGER_H
#define IR_MANAGER_H

#include <Arduino.h>
#include "mbed.h"

class IRManager {
public:
  IRManager(int recvPin, int ledPin, unsigned long uniqueTime);
  void begin();
  void sendID();
  bool hasNewMessage();
  int getReceivedTime();
  void clearMessageFlag();

private:
  int _recvPin;
  mbed::PwmOut _IRLED;
  mbed::Timer _pulseTimer;
  mbed::Timeout _stopPulseTimeout;

  volatile bool _sending;
  volatile bool _messageReceived;
  volatile int _readUserTime;

  static IRManager* _instance;
  static void isrWrapper();
  static void timeoutWrapper();
  
  void handleInterrupt();
  void stopPulse();
};
#endif