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
mbed::Ticker TXTicker; // BLE packets: fixed 100 ms, so the laptop gets a steady 10 Hz
volatile bool flagSendTX = false;
void triggerTX() { flagSendTX = true; }

// IR beacons have their own timer so they can be jittered. Each beacon is scheduled on an absolute deadline:
// previous + 100 ms - J/2 + U(0, J), so the mean stays 10 Hz and no error accumulates. J = 0 is the original
// fixed period. Random offsets stop two nodes from overlapping, or blanking each other while transmitting,
// beacon after beacon for minutes.
const long BEACON_PERIOD_US = 100000;
const long MAX_JITTER_MS = 50;
mbed::Timeout beaconTimeout;
mbed::HighResClock::time_point nextBeaconTime;
volatile bool flagBeacon = false;
volatile long beaconJitterMs = 20;
uint32_t rngState = 1;

uint32_t nextRandom() { // xorshift32, seeded per chip in setup() so no two nodes share a sequence
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

void onBeacon();

void scheduleBeacon() {
  long spanUs = beaconJitterMs * 1000;
  long offsetUs = spanUs > 0 ? (long)(nextRandom() % (uint32_t)(spanUs + 1)) - spanUs / 2 : 0;
  nextBeaconTime += std::chrono::microseconds(BEACON_PERIOD_US + offsetUs);
  beaconTimeout.attach_absolute(&onBeacon, nextBeaconTime);
}

void onBeacon() {
  flagBeacon = true;
  scheduleBeacon();
}

// Restart the beacon schedule so the next beacon goes out offsetMs from now. Sent to two nodes back to back
// over USB (a few ms apart at most), it lines their beacons up for E4 without a sync wire.
void syncBeacons(long offsetMs) {
  mbed::CriticalSectionLock lock;
  beaconTimeout.detach();
  nextBeaconTime = mbed::HighResClock::now() + std::chrono::milliseconds(offsetMs);
  beaconTimeout.attach_absolute(&onBeacon, nextBeaconTime);
}

// Track whether the assigned ID has been pushed to IRManager for pulse width configuration
bool identityConfigured = false;

// --- Bench commands over USB serial, one per line ---
// DUTY <percent> : IR carrier duty (LED on-time per 28 us period), 7-50 %
// CARRIER [ms]   : continuous carrier at the current duty for ms (default 3000, max 5000), for a meter check
// JITTER <ms>    : beacon jitter J, 0-50 ms (default 20 = 100 +/- 10 ms); 0 = fixed 100 ms period
// TX 0|1         : stop / resume this node's own IR beacons (default on); a silent receiver never blanks itself
// ID <n> | ID OFF: bench mode: use IR ID n (0-15) and beacon without a Bluetooth connection; OFF returns to BLE control
// WIDTH <b> <s>  : width code (default): one burst of b + s * ID us (b >= 280, s 20-1000)
// INTERVAL <m> <s>: interval code: mark of m carrier cycles (10-35), gap of m cycles + s * ID us, mark;
//                  receivers log both marks as R lines and frames are decoded from them afterwards
// SYNC <ms>      : next beacon goes out ms (0-1000) from now; later beacons follow the usual schedule
// RAW 0|1        : stop / resume the raw event lines (default on)
// INFO           : I,<chip uid>,<IR ID>,<carrier on us>,<carrier period us>,<BLE state>,<jitter ms>,<TX 0/1>,<bench 0/1>,
//                  <code W/I>,<base us or mark cycles>,<step us>
// Raw event lines: R,<start us>,<width us>,<decoded ID or -1> for each received pulse,
//                  T,<start us>,<burst us> for each own transmission, D,<n> if n events were lost
String serialLine;
bool rawLogging = true;
bool beaconsEnabled = true;
bool benchMode = false; // beacon with a serial-assigned ID, no Bluetooth needed (bench experiments)
int benchId = -1;

void printCarrierDuty() {
  Serial.println("IR carrier duty: " + String(ir.getCarrierOnUs()) + "/" + String(ir.getCarrierPeriodUs()) +
                 " us on (" + String(100.0f * ir.getCarrierOnUs() / ir.getCarrierPeriodUs(), 1) + " %)");
}

void printInfo() {
  Serial.println("I," + BLEManager::nodeUid() + "," + String(benchMode ? benchId : ble.getAssignedID()) + "," +
                 String(ir.getCarrierOnUs()) + "," + String(ir.getCarrierPeriodUs()) + "," + String((int)ble.getState()) +
                 "," + String(beaconJitterMs) + "," + String(beaconsEnabled ? 1 : 0) + "," + String(benchMode ? 1 : 0) +
                 "," + String(ir.isIntervalCode() ? "I" : "W") + "," + String(ir.getCodeBase()) + "," + String(ir.getCodeStepUs()));
}

void printCode() {
  if (ir.isIntervalCode()) {
    Serial.println("Code: interval, marks " + String(ir.getCodeBase()) + " cycles (" +
                   String(ir.getCodeBase() * ir.getCarrierPeriodUs()) + " us), step " + String(ir.getCodeStepUs()) +
                   " us, this node's frame " + String(ir.getFrameUs()) + " us");
  } else {
    Serial.println("Code: width, base " + String(ir.getCodeBase()) + " us, step " + String(ir.getCodeStepUs()) +
                   " us, this node's burst " + String(ir.getFrameUs()) + " us");
  }
}

void setBeaconJitter(long ms) {
  beaconJitterMs = constrain(ms, 0, MAX_JITTER_MS);
  if (beaconJitterMs == 0) {
    Serial.println("Beacon timing: fixed 100 ms");
  } else {
    Serial.println("Beacon timing: 100 ms +/- " + String(beaconJitterMs / 2.0f, 1) + " ms (random), jitter=" +
                   String(beaconJitterMs));
  }
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
    } else if (serialLine.startsWith("JITTER ")) {
      setBeaconJitter(serialLine.substring(7).toInt());
    } else if (serialLine == "ID OFF") {
      benchMode = false;
      Serial.println("Bench mode: off (ID and beacons follow Bluetooth again)");
    } else if (serialLine.startsWith("ID ")) {
      benchId = constrain(serialLine.substring(3).toInt(), 0, 15);
      ir.setIdentity(benchId);
      benchMode = true;
      Serial.println("Bench mode: IR ID " + String(benchId) + ", beaconing without Bluetooth");
    } else if (serialLine.startsWith("WIDTH ") || serialLine.startsWith("INTERVAL ")) {
      bool interval = serialLine.startsWith("INTERVAL ");
      int first = interval ? 9 : 6;
      int space = serialLine.indexOf(' ', first);
      if (space < 0) {
        Serial.println(interval ? "Usage: INTERVAL <mark cycles> <step us>" : "Usage: WIDTH <base us> <step us>");
      } else {
        int a = serialLine.substring(first, space).toInt();
        int s = serialLine.substring(space + 1).toInt();
        if (interval) ir.setIntervalCode(a, s); else ir.setWidthCode(a, s);
        printCode();
      }
    } else if (serialLine.startsWith("SYNC ")) {
      long ms = constrain(serialLine.substring(5).toInt(), 0, 1000);
      syncBeacons(ms);
      Serial.println("Beacons re-synced: next in " + String(ms) + " ms");
    } else if (serialLine == "TX 0" || serialLine == "TX 1") {
      beaconsEnabled = serialLine.endsWith("1");
      Serial.println(beaconsEnabled ? "Beacons: on" : "Beacons: off (receive only)");
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
               (int)event.id); // decoded in the receiver ISR (interval code: on a frame's second mark)
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
  rngState = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1] ^ micros();
  if (rngState == 0) rngState = 1;
  setBeaconJitter(beaconJitterMs);

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
  nextBeaconTime = mbed::HighResClock::now();
  scheduleBeacon();
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
  int jitterMs;
  if (ble.takeBeaconJitterUpdate(jitterMs)) { // optional fourth config byte
    setBeaconJitter(jitterMs);
  }

  // Configure the IRManager with the assigned ID when BLE is streaming
  if( ble.getState() == STATE_STREAMING && !identityConfigured) {
    if (!benchMode) ir.setIdentity(ble.getAssignedID()); // a bench ID takes priority
    identityConfigured = true;
    Serial.println("IR Identity Configured for User ID: " + String(ble.getAssignedID()));
  } // Reset the ID if streaming is interrupted or disconnected
  if( ble.getState() != STATE_STREAMING ) {
    identityConfigured = false;
  }

  // IR beacon on its own (optionally jittered) schedule; only while streaming, after the ID is set, as before
  if (flagBeacon) {
    flagBeacon = false;
    if ((ble.getState() == STATE_STREAMING || benchMode) && beaconsEnabled) ir.sendID();
  }

  if (ble.getState() == STATE_STREAMING) {

    // 1. Accumulate IR glance detections asynchronously
    if (ble.isIRRequested() && ir.hasNewMessage()) {
      int identifiedUser = ir.getReceivedIdentity(); // decoded in the receiver ISR (width or interval code)

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

      // Send the packed 30-byte payload instantly to the laptop
      ble.sendPacket(currentPacket);

      // Reset the tracking bitmask for the next timing frame
      currentPacket.irLookedBitmask = 0;
    }
  }
}