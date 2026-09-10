#include <FlexCAN_T4.h>
#include <SPI.h>
#include "config.h" 
#include <Adafruit_MAX31855.h>
#include <SoftwareSerial.h>


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can0;
SoftwareSerial nextion(16, 17);


typedef void (*State)();
State state = nullptr;

typedef enum {
  MSG_ID_PACK_DATA = 0x0A0,
  MSG_ID_PACK_STATUS = 0x0A1,
  MSG_ID_VSM_STATE = 0x0A2,
  MSG_ID_FAULT_FLAGS = 0x0A4


} CANMessageID;

typedef enum {
  STATUS_VOLT_TOO_HIGH_MASK = 0x0020u,
  STATUS_REDUN_SUPPLY_MASK = 0x1000u,
  VSM_STATE_MASK = 0x0Fu,
  VSM_STATE_DRIVE_ENABLED = 5u,
  VSM_STATE_FAULT = 7u,
  FAULT_FLAGS_BIT0 = 0x01u,
  DTC_COUNT = 32

} DTC;

struct DTCError{
  DTC dtc;
  bool ative;
  void (*check)(void);
};

void handle(void) {
  //Handle dtc errors

}

const DTCError dtcTable[DTC_COUNT] = {
  {STATUS_VOLT_TOO_HIGH_MASK, false, handle},
  {STATUS_REDUN_SUPPLY_MASK, false, handle},
  // {VSM_STATE_MASK, false, nullptr},
  // {VSM_STATE_DRIVE_ENABLED, false, nullptr},
  // {VSM_STATE_FAULT, false, nullptr},
  // {FAULT_FLAGS_BIT0, false, nullptr}
};





struct PackModule{

  volatile uint8_t voltage;
  volatile uint8_t avgTemp;
  volatile uint8_t soc;
  volatile uint8_t maxTemp;
  volatile uint8_t minTemp;


  
}; 



static PackModule pack_data_ptr;
static PackModule *pack = &pack_data_ptr;


struct VCUModule{
  volatile uint8_t vcuState;
  volatile uint8_t vcuFault;
  volatile uint8_t vcuError;
  volatile uint8_t vcuWarning;

  volatile uint8_t canbusOperational;
};

static VCUModule vcu_ptr;
static VCUModule *vcu = &vcu_ptr;

struct PumpModule{
  volatile float ref = 2.58f;
  volatile float bias = 0.555f;
  volatile int rawADC = 0;

  volatile float adc_scale;
  volatile uint8_t current;

  volatile uint8_t biasCurrent;

  volatile uint8_t temperature;
  volatile uint8_t pwmCmd;
  volatile float target_I;
  volatile float error;

  volatile bool ampSensReady = false;
  volatile bool tempSensReady = false;

  volatile unsigned long five_seconds = 0;
  volatile unsigned long short_time = 0;
  volatile unsigned long last_step_ms = 0;
};

static PumpModule pump_ptr;
static PumpModule *pump = &pump_ptr;

static inline void pumpModuleInit() {

  pump->current = 0;
  pump->temperature = 0;

  pinMode(PWM_PIN, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(PWR_LED, OUTPUT);

  analogWriteResolution(8);
  analogWriteFrequency(PWM_PIN, 1000);
  analogWrite(PWM_PIN, 0);

  pump->ampSensReady = true;
}

static inline void pumpModuleUpdate() {

  if (!pump->ampSensReady) return;

  pump->rawADC = analogRead(CURRENT_SENSOR_PIN);
  pump->adc_scale = (float)pump->rawADC / 1023;
  pump->current = (pump->ref - (pump->ref * pump->adc_scale)) - 0.0185;
  pump->biasCurrent = pump->current - pump->bias;
  pump->five_seconds = millis();
  pump->short_time = millis();
  pump->last_step_ms = 0;
  
  if (millis() - pump->five_seconds >= 5000) {
    pump->target_I = pump->current;
    pump->five_seconds = millis();
  }

  if (millis() - pump->short_time >= 50) {
    pump->error = pump->target_I - pump->current;
    if (pump->error < 0 ) pump->error *= -1;
    if (pump->error > 0.5) {
      Serial.print("DO SOMETHING");
    }
    pump->short_time = millis();

  }

  if (pump->pwmCmd < 190 && (millis() - pump->last_step_ms >= 3000)) {

    pump->pwmCmd += 30;
    pump->pwmCmd = constrain(pump->pwmCmd, PWM_MAX, PWM_MIN);
    analogWrite(PWM_PIN, pump->pwmCmd);
    pump->last_step_ms = millis();

  }  
}

void displayWrite(const char *component, const char *label, int value, const char *unit) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s.txt=\"%s %d %s\"", component, label, value, unit);
  nextion.print(buf);
  nextion.write(0xFF);
  nextion.write(0xFF);
  nextion.write(0xFF);
}


static void onCanFrame(const CAN_message_t &msg) {
  if (msg.id == MSG_ID_PACK_DATA) {
    pack->voltage = msg.buf[1] / 10;
    pack->avgTemp = msg.buf[0]; 
    pack->soc = msg.buf[2] / 2;
    pack->maxTemp = msg.buf[3];
    pack->minTemp = msg.buf[4];
    state = pumpModuleUpdate;
  }

  else if (msg.id == 0x0A3) {
    uint32_t raw = msg.buf[0];
    uint32_t raw2 = msg.buf[1];
    Serial.println(raw);
    Serial.println(raw2);
  }

  else if (msg.id == MSG_ID_PACK_STATUS) {
    uint8_t byte0 = msg.buf[0];
    uint8_t byte1 = msg.buf[1];

    if (byte0 & STATUS_VOLT_TOO_HIGH_MASK) {
      displayWrite("t4", "VOLT TOO HIGH", 0, "X");
    }
    if (byte1 & STATUS_REDUN_SUPPLY_MASK) {
      displayWrite("t4", "REDUN SUPPLY", 0, "X");
    }
  }

  else { 
    displayWrite("t0","CAN_DC", 1, "C");
    displayWrite("t3", "X", 1, "V");
    displayWrite("t2", "X", 1, "%");
    displayWrite("t10", "X ", 1, "C");
    displayWrite("t8", "X ", 1, "C");
  
  }
  


}




// static bool tempSensorRdy = false;
// static uint32_t maxRawData        = 0;
// static float    thermocoupleTempC = 0.0f;
// static float    internalTempC     = 0.0f;
// static bool     faultOpenCircuit  = false;
// static bool     faultShortGND     = false;
// static bool     faultShortVCC     = false;
// static bool     thermocoupleFault = false;
static bool     thermocoupleReady = false;
// static uint32_t lastReadMillis    = 0;

#define SCLK 13
#define MISO 12
#define MAX31855_CS_PIN 10

Adafruit_MAX31855 tc(SCLK, MAX31855_CS_PIN, MISO);
static void temperatureController() {
  displayWrite("t4", "Hello", 0, "X");
  double tempC = tc.readCelsius();
  
  if (isnan(tempC)) {
    displayWrite("t2", "temp", tempC, "X");
  } else {
    double tempF = tempC * 9.0 / 5.0 + 32.0;
    int tempF_int = (int)(tempF + 0.5);
    displayWrite("t2", "Temperature", tempC, "C");
    Serial.println(tempC);
    //sendToNextion("n4", tempF_int);
  }
  
  // // Send to Nextion n3 (integer amps)
  // int ampsInt = (int)(ampsFiltered + 0.5f);
  // sendToNextion("n3", ampsInt);
}



void setup() {
  Serial.begin(NEXTION_BAUD);
  nextion.begin(NEXTION_BAUD);

  can0.begin();
  can0.setBaudRate(CAN_BAUD_RATE);
  can0.setMaxMB(16);
  can0.enableFIFO();
  can0.enableFIFOInterrupt();
  can0.onReceive(onCanFrame);
  can0.mailboxStatus();
  displayWrite("t4", "Offline", 0, "X");
}

void loop() {
  can0.events();
  
 
}
