#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include "IRManager.h"

// Pin & Timing Config
const int RECV_PIN = 9; // Digital Pin Number
const int LED_PIN = 8;  // Digital Pin Number
const unsigned long MY_UNIQUE_TIME = 1000; // Unique pulse duration in microseconds (e.g., 1400us for user ID 2, 1600us for user ID 3, etc.)

// Instantiate IRManager
IRManager ir(RECV_PIN, LED_PIN, MY_UNIQUE_TIME);

// Scalable User Tracking
const int MAX_USERS = 5;
bool userLooked[MAX_USERS] = {false};

unsigned long currentTimeLoop = 0;
unsigned long previousSend = 0;
const unsigned long intervalSend = 100000;

// IMU variables
float ownx, owny, ownz;
unsigned long previousMag = 0;
const unsigned long intervalMag = 1000000;

void ReadID(int readUserTime) {
  // Filter out noise completely outside expected bounds
  if (readUserTime < 900 || readUserTime > 2500) {
    return;
  }

  int userID = (readUserTime - 900) / 200; // Map to user ID (0-4)

  if( userID >= 0 && userID < MAX_USERS) {
    userLooked[userID] = true; // Mark the user as looked at
  }
}

void setup() {
  Serial.begin(115200);

  ir.begin();

  if(!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
}

void loop() {

  currentTimeLoop = micros();

  // Handle incoming IR messages
  if (ir.hasMessage()) {
    ReadID(ir.getReceivedTime());
    ir.acknowledgeMessage();
  }

  // Poll Magnetometer
  if( currentTimeLoop - previousMag >= intervalMag) {
    IMU.readMagneticField(ownx, owny, ownz); // TODO: Ensure the axiis are properly aligned with the devices 
    previousMag = currentTimeLoop;
  }

  // Transmit ID and Broadcast Data at 10Hz
  if( currentTimeLoop - previousSend >= intervalSend) {
    ir.sendID();
    previousSend = currentTimeLoop;

    // Dynamic Broadcast
    unsigned int binaryDataWhoLooked = 0;
    bool anyoneLooked = false;
    for( int i = 0; i < MAX_USERS; i++) {
      if(userLooked[i]) {
        bitWrite(binaryDataWhoLooked, i, 1); 
        anyoneLooked = true;
      }
    }

    //Only output to Serial if there is new data
    if( anyoneLooked) {
      Serial.println(binaryDataWhoLooked, BIN);

      // Reset the user looked array for the next loop
      for( int i = 0; i < MAX_USERS; i++) {
        userLooked[i] = false;
      }
    }
  }
}
