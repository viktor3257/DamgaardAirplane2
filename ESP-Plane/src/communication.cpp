#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include "communication.h"
#include "sensors.h"

// ========================= SX1262 wiring (airplane) =========================
static constexpr int kCsPin    = 5;
static constexpr int kDio1Pin  = 34;
static constexpr int kResetPin = 2;
static constexpr int kBusyPin  = 35;
static constexpr int kRxEnPin  = 4;

static SX1262 radio = new Module(kCsPin, kDio1Pin, kResetPin, kBusyPin);

static inline void goRx() { digitalWrite(kRxEnPin, HIGH); }
static inline void goTx() { digitalWrite(kRxEnPin, LOW);  }

// ============================== Protocol ====================================
static constexpr uint8_t kControlId0   = 'C';
static constexpr uint8_t kControlId1   = 'T';
static constexpr uint8_t kSettingsId0  = 'S';
static constexpr uint8_t kSettingsId1  = 'T';
static constexpr uint8_t kTelemetryId0 = 'T';
static constexpr uint8_t kTelemetryId1 = 'L';

static constexpr unsigned long kPlaneTimeoutMs    = 2000;
static constexpr unsigned long kTelemetryPeriodMs = 1000;

struct __attribute__((packed)) ControlPacket {
  uint8_t id0;
  uint8_t id1;
  uint8_t mode;
  uint8_t throttle;
  uint8_t pitch;
  uint8_t roll;
};
static_assert(sizeof(ControlPacket) == 6, "ControlPacket size");

struct __attribute__((packed)) SettingsPacket {
  uint8_t id0;
  uint8_t id1;
  uint8_t initialBattery;
  uint8_t responsiveness;
  int8_t  trim;
  uint8_t autoAltitude;
  uint8_t autoTaxiSpeed;
  uint8_t autoCircleSpeed;
};
static_assert(sizeof(SettingsPacket) == 8, "SettingsPacket size");

struct __attribute__((packed)) TelemetryPacket {
  uint8_t id0;
  uint8_t id1;
  uint8_t mode;
  uint8_t battery;
  int16_t alt_dm;
  uint16_t air_cmps;
  int16_t pitch_cd;
  int16_t roll_cd;
};
static_assert(sizeof(TelemetryPacket) == 12, "TelemetryPacket size");

// ============================== Public state =================================
bool    g_radioLinkActive = false;
uint8_t g_airplaneMode    = AIRPLANE_MODE_NEUTRAL;

float g_controlKnob1 = 0.0f;
float g_controlKnob2 = 0.0f;
float g_controlKnob3 = 0.0f;
float g_controlJoyRX = 0.0f;
float g_controlJoyRY = 0.0f;

int g_settingInitialBatteryPercent = 0;
int g_settingResponsiveness        = 0;
int g_settingTrim                  = 0;
int g_settingAutoAltitude          = 0;
int g_settingAutoTaxiSpeed         = 0;
int g_settingAutoCircleSpeed       = 0;

bool  g_airplaneReceivesDataFromController = false;

float g_radioTxPerSec = 0.0f;
float g_radioRxPerSec = 0.0f;

// ============================== Internal state ===============================
static volatile bool s_irqTxDone = false;
static volatile bool s_irqRxDone = false;

static volatile bool s_txBusy     = false;
static volatile uint32_t s_txStartUs = 0;

static uint32_t s_txCount = 0;
static uint32_t s_rxCount = 0;

static unsigned long s_lastControlMs   = 0;
static unsigned long s_lastTelemetryMs = 0;
static unsigned long s_lastRateMs      = 0;

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

static void processControl(const ControlPacket &pkt) {
  g_airplaneMode = pkt.mode;
  g_controlKnob1 = static_cast<float>(pkt.throttle) / 255.0f;
  g_controlJoyRY = static_cast<float>(pkt.pitch) / 255.0f;
  g_controlJoyRX = static_cast<float>(pkt.roll) / 255.0f;
  g_airplaneReceivesDataFromController = true;
  g_radioLinkActive = true;
  s_lastControlMs = millis();
}

static void processSettings(const SettingsPacket &pkt) {
  g_settingInitialBatteryPercent = static_cast<int>(pkt.initialBattery);
  g_settingResponsiveness        = static_cast<int>(pkt.responsiveness);
  g_settingTrim                  = static_cast<int>(pkt.trim);
  g_settingAutoAltitude          = static_cast<int>(pkt.autoAltitude);
  g_settingAutoTaxiSpeed         = static_cast<int>(pkt.autoTaxiSpeed);
  g_settingAutoCircleSpeed       = static_cast<int>(pkt.autoCircleSpeed);
}

static bool sendTelemetryPacket() {
  TelemetryPacket pkt{};
  pkt.id0     = kTelemetryId0;
  pkt.id1     = kTelemetryId1;
  pkt.mode    = g_airplaneMode;
  pkt.battery = static_cast<uint8_t>(lroundf(constrain(g_airplaneBatteryPercent, 0.0f, 100.0f)));
  pkt.alt_dm  = static_cast<int16_t>(lroundf(g_airplaneHeight * 10.0f));
  pkt.air_cmps= static_cast<uint16_t>(lroundf(constrain(g_airplaneAirSpeed * 100.0f, 0.0f, 65535.0f)));
  pkt.pitch_cd= static_cast<int16_t>(lroundf(g_airplanePitch * 100.0f));
  pkt.roll_cd = static_cast<int16_t>(lroundf(g_airplaneRoll * 100.0f));
  return startTransmit(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

static void handleRx() {
  uint8_t buffer[32] = {0};
  size_t len = sizeof(buffer);
  int16_t st = radio.readData(buffer, len);
  beginRx();
  if (st != RADIOLIB_ERR_NONE || len < 2) return;

  if (len == sizeof(ControlPacket)) {
    const ControlPacket *pkt = reinterpret_cast<const ControlPacket*>(buffer);
    if (pkt->id0 == kControlId0 && pkt->id1 == kControlId1) {
      processControl(*pkt);
      s_rxCount++;
    }
  } else if (len == sizeof(SettingsPacket)) {
    const SettingsPacket *pkt = reinterpret_cast<const SettingsPacket*>(buffer);
    if (pkt->id0 == kSettingsId0 && pkt->id1 == kSettingsId1) {
      processSettings(*pkt);
      s_rxCount++;
    }
  }
}

static void updateRadioStats(unsigned long nowMs) {
  if (nowMs - s_lastRateMs < 1000) return;
  static uint32_t lastTxCount = 0;
  static uint32_t lastRxCount = 0;
  uint32_t txDelta = s_txCount - lastTxCount;
  uint32_t rxDelta = s_rxCount - lastRxCount;
  lastTxCount = s_txCount;
  lastRxCount = s_rxCount;
  s_lastRateMs = nowMs;
  g_radioTxPerSec = 0.8f * g_radioTxPerSec + 0.2f * static_cast<float>(txDelta);
  g_radioRxPerSec = 0.8f * g_radioRxPerSec + 0.2f * static_cast<float>(rxDelta);
}

static void updateLinkState(unsigned long nowMs) {
  bool linkOk = (s_lastControlMs != 0) && (nowMs - s_lastControlMs <= kPlaneTimeoutMs);
  g_radioLinkActive = linkOk;
  g_airplaneReceivesDataFromController = linkOk;
  if (!linkOk) {
    g_controlKnob1 = 0.0f;
  }
}

// ------------------------------ Public API ----------------------------------
void initCommunication() {
  SPI.begin(18, 19, 23);
  Serial2.begin(115200);

  pinMode(kRxEnPin, OUTPUT);

  int16_t st = radio.beginFSK(869.525);
  if (st) {
    g_radioLinkActive = false;
    return;
  }

  setFSKProfile();
  radio.setDio1Action(onDio1);

  beginRx();

  s_lastControlMs   = 0;
  s_lastTelemetryMs = 0;
  s_lastRateMs      = millis();
  g_radioLinkActive = false;
  g_airplaneReceivesDataFromController = false;
}

void handleCommunication() {
  unsigned long nowMs = millis();

  if (s_irqTxDone) {
    s_irqTxDone = false;
    s_txBusy = false;
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

  if (!s_txBusy && (nowMs - s_lastTelemetryMs >= kTelemetryPeriodMs)) {
    if (sendTelemetryPacket()) {
      s_lastTelemetryMs = nowMs;
    }
  }

  updateLinkState(nowMs);
  updateRadioStats(nowMs);
}
