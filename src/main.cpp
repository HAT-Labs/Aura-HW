#include <Arduino.h>
#include "Arduino_BMI270_BMM150.h"
#include "IRManager.h"
#include "BLEManager.h"
#include "DataPacket.h"

// --- Hardware Modules ---
IRManager ir(9, 8); // Recv Pin 9, LED Pin 8
BLEManager ble;

// --- System Telemetry Instance ---
SensorPacket currentPacket = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// --- Hardware Interrupt Tickers ---
mbed::Ticker TXTicker;
volatile bool flagSendTX = false;
void triggerTX() { flagSendTX = true; }

// Track whether the assigned ID has been pushed to IRManager for pulse width configuration
bool identityConfigured = false;

// --- Bench commands over USB serial, one per line ---
// DUTY <percent> : IR carrier duty (LED on-time per 28 us period), 7-50 %
// CARRIER [ms]   : continuous carrier at the current duty for ms (default 3000, max 5000), for a meter check
// RAW 0|1        : stop / resume the raw event lines (default on)
// INFO           : I,<chip uid>,<assigned ID>,<carrier on us>,<carrier period us>,<BLE state>
// Raw event lines: R,<start us>,<width us>,<decoded ID or -1> for each received pulse,
//                  T,<start us>,<burst us> for each own transmission, D,<n> if n events were lost
String serialLine;
bool rawLogging = true;

void printCarrierDuty() {
  Serial.println("IR carrier duty: " + String(ir.getCarrierOnUs()) + "/" + String(ir.getCarrierPeriodUs()) +
                 " us on (" + String(100.0f * ir.getCarrierOnUs() / ir.getCarrierPeriodUs(), 1) + " %)");
}

void printInfo() {
  Serial.println("I," + BLEManager::nodeUid() + "," + String(ble.getAssignedID()) + "," +
                 String(ir.getCarrierOnUs()) + "," + String(ir.getCarrierPeriodUs()) + "," + String((int)ble.getState()));
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n') { serialLine += c; continue; }
    serialLine.trim();
    serialLine.toUpperCase();
    if (serialLine.startsWith("DUTY ")) {
      ir.setCarrierDuty(serialLine.substring(5).toInt());
      printCarrierDuty();
    } else if (serialLine == "CARRIER" || serialLine.startsWith("CARRIER ")) {
      long ms = serialLine.length() > 8 ? serialLine.substring(8).toInt() : 3000;
      ms = constrain(ms, 1, 5000);
      ir.startCarrierTest(ms);
      Serial.println("Carrier test: " + String(ms) + " ms continuous, " + String(ir.getCarrierOnUs()) + "/" +
                     String(ir.getCarrierPeriodUs()) + " us on");
    } else if (serialLine == "RAW 0" || serialLine == "RAW 1") {
      rawLogging = serialLine.endsWith("1");
    } else if (serialLine == "INFO") {
      printInfo();
    } else if (serialLine.length() > 0) {
      Serial.println("Unknown command: " + serialLine);
    }
    serialLine = "";
  }
}

void printRawEvents() {
  IRManager::RawEvent event;
  char line[48];
  for (int n = 0; n < 16 && ir.popRawEvent(event); n++) {
    if (!rawLogging) continue;
    if (event.kind == 'R') {
      snprintf(line, sizeof(line), "R,%lu,%lu,%d", (unsigned long)event.tUs, (unsigned long)event.widthUs,
               ir.readIdentity((int)event.widthUs));
    } else {
      snprintf(line, sizeof(line), "T,%lu,%lu", (unsigned long)event.tUs, (unsigned long)event.widthUs);
    }
    Serial.println(line);
  }
  uint32_t dropped = ir.takeDroppedEvents();
  if (dropped > 0 && rawLogging) Serial.println("D," + String(dropped));
}

void setup() {
  Serial.begin(115200);

  delay(2000);
  Serial.println("Starting Boot Sequence...");

  ir.begin();
  Serial.println("IR Initialized.");
  printCarrierDuty();

  if( !IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
  Serial.println("IMU Initialized.");

  if( !ble.begin()) {
    Serial.println("Failed to initialize BLE!");
    while(1);
  }
  Serial.println("BLE Initialized.");
  printInfo();

  // Set streaming transmission rate (e.g., 10Hz / every 100ms)
  TXTicker.attach(&triggerTX, 0.1); 
}

void loop() {
  ble.update();
  handleSerialCommands();
  printRawEvents();

  // Apply a carrier duty sent by the laptop (optional third config byte). The latest command wins.
  int dutyPercent;
  if (ble.takeCarrierDutyUpdate(dutyPercent)) {
    ir.setCarrierDuty(dutyPercent);
    printCarrierDuty();
  }

  // Configure the IRManager with the assigned ID when BLE is streaming
  if( ble.getState() == STATE_STREAMING && !identityConfigured) {
    ir.setIdentity(ble.getAssignedID());
    identityConfigured = true;
    Serial.println("IR Identity Configured for User ID: " + String(ble.getAssignedID()));
  } // Reset the ID if streaming is interrupted or disconnected
  if( ble.getState() != STATE_STREAMING ) {
    identityConfigured = false;
  }
  
  if (ble.getState() == STATE_STREAMING) {

    // 1. Accumulate IR glance detections asynchronously
    if (ble.isIRRequested() && ir.hasNewMessage()) {
      int pulseDuration = ir.getReceivedTime();
      int identifiedUser = ir.readIdentity(pulseDuration); 

      if (identifiedUser >= 0 && identifiedUser < 16 && ble.getAssignedID() != identifiedUser) {
        bitWrite(currentPacket.irLookedBitmask, identifiedUser, 1);
      }
      ir.clearMessageFlag();
    }

    // 2. Synchronous Packet Compilation and Transmission
    if (flagSendTX) {
      flagSendTX = false; // Clear task flag

      // Assign the precise hardware timestamp
      currentPacket.timestamp = millis(); 

      // Poll IMU data safely if requested
      if (ble.isIMURequested()) {
        // Create naturally-aligned stack variables
        float ax, ay, az;

        // Read from the IMU into aligned variables (Safe for references)
        IMU.readAcceleration(ax, ay, az);

        // Safely copy the values into your packed struct
        currentPacket.accX = ax;
        currentPacket.accY = ay;
        currentPacket.accZ = az;
      } else {
        // If IMU data is not requested, zero it out
        currentPacket.accX = 0.0f;
        currentPacket.accY = 0.0f;
        currentPacket.accZ = 0.0f;
      }

      if (ble.isMagRequested()) {
        // If magnetometer data is requested, ensure it's read and included
        float mx, my, mz;

        // Read from the magnetometer
        IMU.readMagneticField(mx, my, mz);

        // Pack the magnetometer data into the struct
        currentPacket.magX = mx;
        currentPacket.magY = my;
        currentPacket.magZ = mz;
      } else {
        // If magnetometer data is not requested, zero it out
        currentPacket.magX = 0.0f;
        currentPacket.magY = 0.0f;
        currentPacket.magZ = 0.0f;
      }

      if (ble.isGyroRequested()) {
        // If gyro data is requested, ensure it's read and included
        float gx, gy, gz;

        // Read from the gyroscope
        IMU.readGyroscope(gx, gy, gz);

        // Pack the gyroscope data into the struct
        currentPacket.gyroX = gx;
        currentPacket.gyroY = gy;
        currentPacket.gyroZ = gz;
      } else {
        // If gyroscope data is not requested, zero it out
        currentPacket.gyroX = 0.0f;
        currentPacket.gyroY = 0.0f;
        currentPacket.gyroZ = 0.0f;
      }

      // Simultaneously blast the IR identity pulse 
      ir.sendID();

      // Send the packed 30-byte payload instantly to the laptop
      ble.sendPacket(currentPacket);

      // Reset the tracking bitmask for the next timing frame
      currentPacket.irLookedBitmask = 0;
    }
  }
}