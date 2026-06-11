#ifndef DATA_PACKET_H
#define DATA_PACKET_H

#include <Arduino.h>

// Forcing 1-byte alignment so the laptop reads the exact raw byte offset
struct __attribute__((__packed__)) SensorPacket {
  uint32_t timestamp;       // 4 bytes (millis or micros)
  uint16_t irLookedBitmask; // 2 bytes (supports tracking up to 16 distinct users)
  float accX, accY, accZ;   // 12 bytes (3 * 4 bytes)
  float magX, magY, magZ;   // 12 bytes (3 * 4 bytes)
}; // Total packet size = 30 bytes

#endif