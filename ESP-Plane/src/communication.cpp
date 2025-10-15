#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include <cstring>
#include "communication.h"
#include "sensors.h"

// ========================= SX1262 wiring (airplane) =========================
static constexpr int kCsPin    = 5;
static constexpr int kDio1Pin  = 34;
static constexpr int kResetPin = 2;
static constexpr int kBusyPin  = 35;
static constexpr int kRxEnPin  = 4;

static SX1262 radio = new Module(kCsPin, kDio1Pin, kResetPin, kBusyPin);

// RF switch helpers
static inline void goRx() { digitalWrite(kRxEnPin, HIGH); }
static inline void goTx() { digitalWrite(kRxEnPin, LOW);  }

// ============================== Goals & policy ===============================
// Must match controller cadence
static constexpr uint32_t kSuperframeUs   = 50000;
// ACK/Telemetry timing (same frame)
static constexpr uint32_t kAckDelayUs     = 6000;   // must match controller

// TX hang safety
static constexpr uint32_t kTxHangTimeoutUs = 20000; // 20 ms watchdog

// DC governor (plane has its own limit)
static constexpr float     kDcTarget       = 0.06f;
static constexpr float     kToASafety      = 1.30f;
static constexpr uint32_t  kDcBucketCapUs  = (uint32_t)(kDcTarget * 60.0f * 1000000.0f);
static constexpr uint32_t  kDcRefillUsPerMs= (uint32_t)(kDcTarget * 1000.0f);

// Link supervision
static const unsigned long kConnectionTimeoutMs = 800;

// ============================== Packet types/format ==========================
enum : uint8_t {
  PKT_CONTROL   = 0x01,
  PKT_SETTINGS  = 0x02,
  PKT_TELEMETRY = 0x03,
  PKT_ACK       = 0x04
};

enum : uint8_t {
  FLG_TLM_REQ   = 1 << 0,
};

// Fixed-point scaling
static constexpr float kInvJoyScale  = 1.0f / 127.0f;
static constexpr float kInvThrot     = 1.0f / 255.0f;

// Packets (must mirror controller)
struct __attribute__((packed)) ControlPacket {
  uint8_t type;
  uint8_t seq;
  uint8_t flags;
  int8_t  pitch;
  int8_t  roll;
  uint8_t throttle;
};
static_assert(sizeof(ControlPacket) == 6, "ControlPacket size");

struct __attribute__((packed)) SettingsPacket {
  uint8_t type;
  uint8_t seq;
  uint8_t flags;
  uint8_t initialBattery;
  uint8_t responsiveness;
  int8_t  trim;
  uint8_t autoAltitude;
  uint8_t autoTaxiSpeed;
  uint8_t autoCircleSpeed;
  uint8_t desiredMode;
  uint8_t switch1;   // mirror controller
  uint8_t switch2;   // mirror controller
};
static_assert(sizeof(SettingsPacket) == 12, "SettingsPacket size");

struct __attribute__((packed)) AckPacket {
  uint8_t type;
  uint8_t ackSeq;
  int8_t  rssi;
  uint8_t status;
};
static_assert(sizeof(AckPacket) == 4, "AckPacket size");

struct __attribute__((packed)) TelemetryPacket {
  uint8_t type;
  uint8_t ackSeq;
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

// Control inputs (decoded from controller)
float g_controlKnob1 = 0.0f;
float g_controlKnob2 = 0.0f;
float g_controlKnob3 = 0.0f;
float g_controlJoyRX = 0.0f;
float g_controlJoyRY = 0.0f;

// Settings
int g_settingInitialBatteryPercent = 0;
int g_settingResponsiveness        = 0;
int g_settingTrim                  = 0;
int g_settingAutoAltitude          = 0;
int g_settingAutoTaxiSpeed         = 0;
int g_settingAutoCircleSpeed       = 0;
// if you need switches later, store them too:
int g_settingSwitch1               = 0;
int g_settingSwitch2               = 0;

bool  g_airplaneReceivesDataFromController = false;

// Radio usage metrics (optional for your UI)
float g_radioTxPerSec = 0.0f;
float g_radioRxPerSec = 0.0f;

// ============================== Internal state ===============================
static volatile bool s_irqTxDone = false;
static volatile bool s_irqRxDone = false;

static volatile bool     s_txBusy    = false;
static volatile uint32_t s_txStartUs = 0;
static volatile uint32_t s_lastTxDurUs = 3000;

static int16_t  s_lastRssi = 0;
static uint32_t s_rxCount = 0, s_txCount = 0;

static uint8_t  s_lastCtrlSeq = 255;
static bool     s_ackPending = false;
static uint32_t s_ackDueUs   = 0;

static bool     s_telemetryPending = false;
static uint32_t s_tlmDueUs         = 0;
static unsigned long s_lastTelemetryMs = 0;

// Fixed bitrate (stability)
static constexpr uint8_t  s_rateIdx = 0;

// DC bucket
static uint32_t s_dcCreditUs = kDcBucketCapUs / 2;
static uint32_t s_dcLastRefillMs = 0;
static uint32_t s_toaEmaUs = 3000;

// Link supervision
static unsigned long s_lastPacketTimeMs = 0;

// ISR
#if defined(ESP32)
void IRAM_ATTR onDio1() {
#else
void onDio1() {
#endif
  if (s_txBusy) s_irqTxDone = true;
  else          s_irqRxDone = true;
}

// PHY profile
static void setFSKProfile(uint8_t rateIdx) {
  (void)rateIdx; // fixed 50 kbps for now
  radio.setPreambleLength(16);
  radio.setCRC(true);
  radio.setDataShaping(0.5);
  radio.setOutputPower(17);
  radio.setBitRate(50.0);
  radio.setRxBandwidth(117.3);
  radio.setFrequencyDeviation(25.0);
}

// DC governor
static inline void dcRefill() {
  uint32_t nowMs = millis();
  if (!s_dcLastRefillMs) { s_dcLastRefillMs = nowMs; return; }
  uint32_t dtMs = nowMs - s_dcLastRefillMs;
  if (dtMs == 0) return;
  uint64_t add = (uint64_t)dtMs * (uint64_t)kDcRefillUsPerMs;
  s_dcLastRefillMs = nowMs;
  uint64_t credit = (uint64_t)s_dcCreditUs + add;
  if (credit > kDcBucketCapUs) credit = kDcBucketCapUs;
  s_dcCreditUs = (uint32_t)credit;
}
static inline bool dcCanAfford(uint32_t estUs) { dcRefill(); return s_dcCreditUs >= estUs; }
static inline void dcReserve(uint32_t estUs)   { if (estUs > s_dcCreditUs) estUs = s_dcCreditUs; s_dcCreditUs -= estUs; }
static inline uint32_t conservativeCostUs(uint8_t /*len*/) {
  uint32_t cost = (uint32_t)((float)s_toaEmaUs * kToASafety);
  if (cost < 1200) cost = 1200; // allow more sends initially
  return cost;
}
static inline uint32_t conservativeCostUsAck() {
  uint32_t cost = (uint32_t)((float)s_toaEmaUs * kToASafety);
  if (cost < 800) cost = 800;   // small min for ACKs
  return cost;
}

static void beginRx() { goRx(); delayMicroseconds(300); radio.startReceive(); }

// Build & send ACK in its slot
static void scheduleAck(uint8_t seq, uint32_t rxTimeUs) {
  s_lastCtrlSeq = seq;
  s_ackDueUs    = rxTimeUs + kAckDelayUs;
  s_ackPending  = true;
}

// Send telemetry in the SAME slot (replaces ACK)
static inline void scheduleTelemetrySameFrame(uint8_t seq, uint32_t rxTimeUs) {
  s_lastCtrlSeq = seq;
  s_tlmDueUs = rxTimeUs + kAckDelayUs;   // same slot timing as ACK
  s_telemetryPending = true;
  s_ackPending = false;                  // telemetry covers ACK semantics
}

// TX helpers
static bool trySendAck() {
  if (s_txBusy) return false;

  AckPacket a{};
  a.type   = PKT_ACK;
  a.ackSeq = s_lastCtrlSeq;
  a.rssi   = (int8_t)s_lastRssi;
  a.status = 0x01; // link OK

  uint32_t cost = conservativeCostUsAck();
  if (!dcCanAfford(cost)) return false;

  goTx();
  delayMicroseconds(300);
  s_txStartUs = micros();
  s_txBusy = true;
  int16_t st = radio.startTransmit((uint8_t*)&a, sizeof(a));
  if (st != RADIOLIB_ERR_NONE) {
    s_txBusy = false;
    radio.standby();
    delayMicroseconds(300);
    beginRx();
    return false;
  }
  dcReserve(cost);
  return true;
}

static bool trySendTelemetry() {
  if (s_txBusy) return false;

  TelemetryPacket t{};
  t.type     = PKT_TELEMETRY;
  t.ackSeq   = s_lastCtrlSeq;
  t.mode     = g_airplaneMode;
  t.battery  = (uint8_t)lroundf(constrain(g_airplaneBatteryPercent, 0.0f, 100.0f));
  t.alt_dm   = (int16_t)lroundf(g_airplaneHeight * 10.0f);
  t.air_cmps = (uint16_t)lroundf(constrain(g_airplaneAirSpeed * 100.0f, 0.0f, 65535.0f));
  t.pitch_cd = (int16_t)lroundf(g_airplanePitch * 100.0f);
  t.roll_cd  = (int16_t)lroundf(g_airplaneRoll * 100.0f);

  uint32_t cost = conservativeCostUs(sizeof(t));
  if (!dcCanAfford(cost)) return false;

  goTx();
  delayMicroseconds(300);
  s_txStartUs = micros();
  s_txBusy = true;
  int16_t st = radio.startTransmit((uint8_t*)&t, sizeof(t));
  if (st != RADIOLIB_ERR_NONE) {
    s_txBusy = false;
    radio.standby();
    delayMicroseconds(300);
    beginRx();
    return false;
  }
  dcReserve(cost);
  s_lastTelemetryMs = millis();
  return true;
}

// Decode controller frames
static void processControl(const ControlPacket& p, uint32_t rxTimeUs) {
  g_controlJoyRY = p.pitch    * kInvJoyScale;
  g_controlJoyRX = p.roll     * kInvJoyScale;
  g_controlKnob1 = p.throttle * kInvThrot;

  g_airplaneReceivesDataFromController = true;
  g_radioLinkActive = true;
  s_lastPacketTimeMs = millis();

  // default: prepare an ACK
  scheduleAck(p.seq, rxTimeUs);

  // If telemetry requested, replace ACK with TELEMETRY in the same slot
  if (p.flags & FLG_TLM_REQ) {
    scheduleTelemetrySameFrame(p.seq, rxTimeUs);
  }
}

static void processSettings(const SettingsPacket& s, uint32_t rxTimeUs) {
  g_settingInitialBatteryPercent = (int)s.initialBattery;
  g_settingResponsiveness        = (int)s.responsiveness;
  g_settingTrim                  = (int)s.trim;
  g_settingAutoAltitude          = (int)s.autoAltitude;
  g_settingAutoTaxiSpeed         = (int)s.autoTaxiSpeed;
  g_settingAutoCircleSpeed       = (int)s.autoCircleSpeed;
  g_settingSwitch1               = (int)s.switch1;
  g_settingSwitch2               = (int)s.switch2;
  g_airplaneMode                 = (uint8_t)s.desiredMode;

  scheduleAck(s.seq, rxTimeUs);
}

// ------------------------------ Public API ----------------------------------
void initCommunication() {
  SPI.begin(18, 19, 23);
  Serial2.begin(115200); // optional Pi link

  pinMode(kRxEnPin, OUTPUT);

  int16_t st = radio.beginFSK(869.525);
  if (st) { g_radioLinkActive = false; return; }

  setFSKProfile(s_rateIdx);
  radio.setDio1Action(onDio1);

  beginRx();

  s_dcLastRefillMs = millis();
  s_lastPacketTimeMs = millis();
  g_radioLinkActive = false;
  g_airplaneReceivesDataFromController = false;
}

void handleCommunication() {
  // Handle IRQs
  if (s_irqTxDone) {
    s_irqTxDone = false;
    s_txBusy = false;
    uint32_t endUs = micros();
    s_lastTxDurUs = endUs - s_txStartUs;
    s_toaEmaUs = (uint32_t)(0.8f * s_toaEmaUs + 0.2f * (float)s_lastTxDurUs);
    s_txCount++;
    beginRx();
  }
  if (s_irqRxDone) {
    s_irqRxDone = false;

    // Timestamp EARLY for better slot alignment
    uint32_t rxTimeUs = micros();

    uint8_t rxBuf[32] = {0};
    size_t  len = sizeof(rxBuf);
    int16_t st = radio.readData(rxBuf, len);
    beginRx();
    if (st == RADIOLIB_ERR_NONE && len > 0) {
      s_rxCount++;
      s_lastRssi = (int16_t)radio.getRSSI();
      uint8_t t = rxBuf[0];

      if (t == PKT_CONTROL && len >= sizeof(ControlPacket)) {
        const ControlPacket* p = reinterpret_cast<const ControlPacket*>(rxBuf);
        processControl(*p, rxTimeUs);
      } else if (t == PKT_SETTINGS && len >= sizeof(SettingsPacket)) {
        const SettingsPacket* s = reinterpret_cast<const SettingsPacket*>(rxBuf);
        processSettings(*s, rxTimeUs);
      }
    }
  }

  // TX hang watchdog
  if (s_txBusy) {
    uint32_t dt = micros() - s_txStartUs;
    if ((int32_t)dt > (int32_t)kTxHangTimeoutUs) {
      s_txBusy = false;
      radio.standby();
      delayMicroseconds(300);
      beginRx();
    }
  }

  // Per-second display rates (simple EMA)
  static uint32_t lastTx = 0, lastRx = 0;
  static float txAvg=0, rxAvg=0;
  static unsigned long lastRateMs = millis();
  unsigned long nowMs = millis();
  if (nowMs - lastRateMs >= 1000) {
    uint32_t txD = s_txCount - lastTx;
    uint32_t rxD = s_rxCount - lastRx;
    lastTx = s_txCount; lastRx = s_rxCount; lastRateMs = nowMs;
    txAvg = 0.8f*txAvg + 0.2f*(float)txD;
    rxAvg = 0.8f*rxAvg + 0.2f*(float)rxD;
    g_radioTxPerSec = txAvg;
    g_radioRxPerSec = rxAvg;
  }

  // Link supervision & failsafe handoff
  if ((nowMs - s_lastPacketTimeMs) > kConnectionTimeoutMs) {
    g_radioLinkActive = false;
    g_airplaneReceivesDataFromController = false;
    // Your main control loop should ensure throttle=0 in failsafe
  }

  // TX scheduling — telemetry takes priority over ACK if both align
  uint32_t nowUs = micros();
  if (s_telemetryPending && (int32_t)(nowUs - s_tlmDueUs) >= 0) {
    if (trySendTelemetry()) {
      s_telemetryPending = false;
    }
  } else if (s_ackPending && (int32_t)(nowUs - s_ackDueUs) >= 0) {
    if (trySendAck()) {
      s_ackPending = false;
    }
  }

  // Forward minimal telemetry to Pi over UART @ 1 Hz (optional)
  static unsigned long lastPiMs = 0;
  if (nowMs - lastPiMs >= 1000) {
    lastPiMs = nowMs;
    char line[128];
    snprintf(line, sizeof(line), "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
             (unsigned)g_airplaneMode, g_airplaneHeight, g_airplaneAirSpeed,
             g_airplanePitch, g_airplaneRoll, g_airplaneBatteryPercent, (int)s_lastRssi);
    Serial2.print(line);
  }
}
