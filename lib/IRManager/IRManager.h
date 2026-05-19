#ifndef IR_MANAGER_H
#define IR_MANAGER_H

#include <Arduino.h>
#include <mbed.h>

class IRManager {
    public:
        // Constructor setups up our pins and timings 
        IRManager(int recvPin, int ledPin, int outPin, unsigned long uniqueTime);
        
        void begin();
        void sendID();
        bool hasMessage();
        int getReceivedTime();
        void acknowledgeMessage();

    private:
        int _recvPin;
        int _ledPin;
        int _outPin;
        unsigned long _uniqueTime;

        mbed::PwmOut _IRLED;
        mbed::Timer _pulseTimer;
        mbed::Timeout _stopPulseTimeout;

        volatile bool _sending;
        volatile bool _receiving;
        volatile int _readUserTime;
        bool _outState;

        // The "Static Wraper" trick for ISRs
        static IRManager* _instance;
        static void isrWrapper();
        static void timeoutWrapper();

        // The actual internal logic
        void handleInterrupt();
        void stopPulse();
};
#endif