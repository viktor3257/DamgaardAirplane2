#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include "communication.h"
#include "sensors.h"
#include "display.h"

// ========================= SX1262 wiring (controller) =======================
static constexpr int kCsPin    = 5;
static constexpr int kDio1Pin  = 27;
static constexpr int kResetPin = 26;
static constexpr int kBusyPin  = 25;
static constexpr int kRxEnPin  = 15;

static SX1262 radio = new Module(kCsPin, kDio1Pin, kResetPin, kBusyPin);

static inline void goRx() { digitalWrite(kRxEnPin, HIGH); }
static inline void goTx() { digitalWrite(kRxEnPin, LOW);  }

// ============================== Protocol ====================================
static constexpr uint8_t kControlId   = 'C';
static constexpr uint8_t kSettingsId  = 'S';
static constexpr uint8_t kTelemetryId = 'T';

static constexpr uint32_t kControlPeriodMs   = 100;   // 10 Hz
static constexpr uint32_t kControllerTimeout = 5000;  // ms

struct __attribute__((packed)) ControlPacket {
  uint8_t id;
  uint8_t mode;
  uint8_t throttle;
  uint8_t pitch;
  uint8_t roll;
};
static_assert(sizeof(ControlPacket) == 5, "ControlPacket size");

struct __attribute__((packed)) SettingsPacket {
  uint8_t id;
  uint8_t initialBattery;
  uint8_t responsiveness;
  int8_t  trim;
  uint8_t autoAltitude;
  uint8_t autoTaxiSpeed;
  uint8_t autoCircleSpeed;
};
static_assert(sizeof(SettingsPacket) == 7, "SettingsPacket size");

struct __attribute__((packed)) TelemetryPacket {
  uint8_t id;
  uint8_t mode;
  uint8_t battery;
  int16_t alt_dm;
  uint16_t air_cmps;
  int16_t pitch_cd;
  int16_t roll_cd;
};
static_assert(sizeof(TelemetryPacket) == 11, "TelemetryPacket size");

// ============================== Globals ======================================
bool g_radioLinkActive = false;
uint8_t g_desiredAirplaneMode = AIRPLANE_MODE_NEUTRAL;
bool g_controllerReceivesDataFromPlane = false;

uint8_t g_airplaneMode = AIRPLANE_MODE_NEUTRAL;
float   g_planeTelemetryBatteryPct  = 0.0f;
float   g_planeTelemetryHeightM     = 0.0f;
float   g_planeTelemetryAirspeedMps = 0.0f;

float g_radioTxPerSec = 0.0f;
float g_radioRxPerSec = 0.0f;
float g_radioTxTimeMs = 0.0f;

// ============================== Internal state ===============================
static volatile bool s_irqTxDone = false;
static volatile bool s_irqRxDone = false;

static volatile bool s_txBusy     = false;
static volatile uint32_t s_txStartUs = 0;

static uint32_t s_txCount = 0;
static uint32_t s_rxCount = 0;

static unsigned long s_lastControlSendMs = 0;
static unsigned long s_lastRateUpdateMs  = 0;
static unsigned long s_lastRxPacketMs    = 0;

static bool s_pendingSettings = false;

// ------------------------------ ISR -----------------------------------------
#if defined(ESP32)
void IRAM_ATTR onDio1() {
#else
void onDio1() {
#endif
  if (s_txBusy) s_irqTxDone = true;
  else          s_irqRxDone = true;
}

// ------------------------------ Helpers -------------------------------------
static void setFSKProfile() {
  radio.setPreambleLength(16);
  radio.setCRC(true);
  radio.setDataShaping(0.5);
  radio.setOutputPower(17);
  radio.setBitRate(50.0);
  radio.setRxBandwidth(117.3);
  radio.setFrequencyDeviation(25.0);
}

static void beginRx() {
  goRx();
  delayMicroseconds(300);
  radio.startReceive();
}

static void updateLinkStatus(unsigned long nowMs) {
  bool rxOk = (s_lastRxPacketMs != 0) && (nowMs - s_lastRxPacketMs <= kControllerTimeout);
  g_controllerReceivesDataFromPlane = rxOk;
  g_radioLinkActive = rxOk;
}

static bool startTransmit(const uint8_t *data, size_t len) {
  if (s_txBusy) return false;
  goTx();
  delayMicroseconds(300);
  s_txBusy = true;
  s_txStartUs = micros();
  int16_t st = radio.startTransmit(data, len);
  if (st != RADIOLIB_ERR_NONE) {
    s_txBusy = false;
    radio.standby();
    delayMicroseconds(300);
    beginRx();
    return false;
  }
  return true;
}

static bool sendControlPacket() {
  ControlPacket pkt{};
  pkt.id = kControlId;
  pkt.mode = g_desiredAirplaneMode;
  pkt.throttle = (uint8_t)lroundf(constrain(g_controlKnob1, 0.0f, 1.0f) * 255.0f);
  pkt.pitch    = (uint8_t)lroundf(constrain(g_controlJoyRY, 0.0f, 1.0f) * 255.0f);
  pkt.roll     = (uint8_t)lroundf(constrain(g_controlJoyRX, 0.0f, 1.0f) * 255.0f);
  return startTransmit(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

static bool sendSettingsPacket() {
  SettingsPacket pkt{};
  pkt.id             = kSettingsId;
  pkt.initialBattery = (uint8_t)getInitialBatteryPercent();
  pkt.responsiveness = (uint8_t)getResponsiveness();
  pkt.trim           = (int8_t)getTrim();
  pkt.autoAltitude   = (uint8_t)getAutoAltitude();
  pkt.autoTaxiSpeed  = (uint8_t)getAutoTaxiSpeed();
  pkt.autoCircleSpeed= (uint8_t)getAutoCircleSpeed();
  return startTransmit(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

static void handleTelemetry(const TelemetryPacket &pkt) {
  g_airplaneMode               = pkt.mode;
  g_planeTelemetryBatteryPct   = static_cast<float>(pkt.battery);
  g_planeTelemetryHeightM      = static_cast<float>(pkt.alt_dm) * 0.1f;
  g_planeTelemetryAirspeedMps  = static_cast<float>(pkt.air_cmps) * 0.01f;
  s_lastRxPacketMs             = millis();
}

static void handleRx() {
  uint8_t buffer[32] = {0};
  size_t len = sizeof(buffer);
  int16_t st = radio.readData(buffer, len);
  beginRx();
  if (st != RADIOLIB_ERR_NONE || len < 1) return;

  if (len >= sizeof(TelemetryPacket)) {
    const TelemetryPacket *pkt = reinterpret_cast<const TelemetryPacket*>(buffer);
    if (pkt->id == kTelemetryId) {
      handleTelemetry(*pkt);
      s_rxCount++;
    }
  }
}

static void updateRadioStats(unsigned long nowMs) {
  static uint32_t lastTxCount = 0;
  static uint32_t lastRxCount = 0;
  if (nowMs - s_lastRateUpdateMs < 1000) return;
  uint32_t txDelta = s_txCount - lastTxCount;
  uint32_t rxDelta = s_rxCount - lastRxCount;
  lastTxCount = s_txCount;
  lastRxCount = s_rxCount;
  s_lastRateUpdateMs = nowMs;
  g_radioTxPerSec = 0.8f * g_radioTxPerSec + 0.2f * static_cast<float>(txDelta);
  g_radioRxPerSec = 0.8f * g_radioRxPerSec + 0.2f * static_cast<float>(rxDelta);
}

// ------------------------------ Public API ----------------------------------
void initCommunication() {
  SPI.begin(18, 19, 23);
  pinMode(kRxEnPin, OUTPUT);

  int16_t st = radio.beginFSK(869.525);
  if (st) {
    g_radioLinkActive = false;
    return;
  }

  setFSKProfile();
  radio.setDio1Action(onDio1);

  beginRx();

  s_lastControlSendMs = millis();
  s_lastRateUpdateMs  = millis();
  s_lastRxPacketMs    = 0;
  g_radioLinkActive   = false;
}

void handleCommunication() {
  unsigned long nowMs = millis();

  if (s_irqTxDone) {
    s_irqTxDone = false;
    s_txBusy = false;
    uint32_t txDurationUs = micros() - s_txStartUs;
    g_radioTxTimeMs = txDurationUs / 1000.0f;
    s_txCount++;
    beginRx();
  }

  if (s_irqRxDone) {
    s_irqRxDone = false;
    handleRx();
  }

  if (s_txBusy) {
    uint32_t dt = micros() - s_txStartUs;
    if (dt > 50000) {
      s_txBusy = false;
      radio.standby();
      delayMicroseconds(300);
      beginRx();
    }
  }

  if (button4Pressed()) {
    s_pendingSettings = true;
  }

  if (!s_txBusy) {
    if (s_pendingSettings) {
      if (sendSettingsPacket()) {
        s_pendingSettings = false;
        s_lastControlSendMs = nowMs;
      }
    } else if (nowMs - s_lastControlSendMs >= kControlPeriodMs) {
      if (sendControlPacket()) {
        s_lastControlSendMs = nowMs;
      }
    }
  }

  updateLinkStatus(nowMs);
  updateRadioStats(nowMs);
}
