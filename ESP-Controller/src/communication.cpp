#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include <cstring>
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

// RF switch helpers
static inline void goRx() { digitalWrite(kRxEnPin, HIGH); }
static inline void goTx() { digitalWrite(kRxEnPin, LOW);  }

// ============================== Goals & policy ===============================
// Superframe period for control loop (20 Hz)
static constexpr uint32_t kSuperframeUs   = 50000;   // 50 ms

// Plane response timing inside each superframe (more margin now)
static constexpr uint32_t kGuardUs        = 5000;    // allow PLL/RF switch settle
static constexpr uint32_t kAckDelayUs     = 6000;    // plane responds at this offset
static constexpr uint32_t kAckWindowUs    = 30000;   // widened window for ACK/Telemetry

// Telemetry cadence target
static constexpr uint32_t kTelemetryIntervalMs = 1000;

// TX hang safety
static constexpr uint32_t kTxHangTimeoutUs = 20000;  // 20 ms watchdog

// DC governor: target 6% (legal limit 10% -> safety margin)
static constexpr float     kDcTarget       = 0.06f;
static constexpr float     kToASafety      = 1.30f;  // 30% overhead on measured ToA
static constexpr uint32_t  kDcBucketCapUs  = (uint32_t)(kDcTarget * 60.0f * 1000000.0f); // per-minute cap
static constexpr uint32_t  kDcRefillUsPerMs= (uint32_t)(kDcTarget * 1000.0f);            // 0.06 * 1000 = 60 us/ms

// ============================== Packet types/format ==========================
enum : uint8_t {
  PKT_CONTROL   = 0x01,
  PKT_SETTINGS  = 0x02,
  PKT_TELEMETRY = 0x03,
  PKT_ACK       = 0x04
};

enum : uint8_t {
  FLG_TLM_REQ   = 1 << 0,   // request telemetry this frame
};

// Fixed-point scaling
static constexpr float kJoyScale     = 127.0f;
static constexpr float kInvJoyScale  = 1.0f / 127.0f;
static constexpr float kThrotScale   = 255.0f;
static constexpr float kInvThrot     = 1.0f / 255.0f;

// Very compact control packet (6 bytes)
struct __attribute__((packed)) ControlPacket {
  uint8_t type;     // PKT_CONTROL
  uint8_t seq;      // rolling sequence number
  uint8_t flags;    // FLG_TLM_REQ etc.
  int8_t  pitch;    // [-127..127]
  int8_t  roll;     // [-127..127]
  uint8_t throttle; // [0..255]
};
static_assert(sizeof(ControlPacket) == 6, "ControlPacket size");

// Event-driven settings packet (12 bytes)
struct __attribute__((packed)) SettingsPacket {
  uint8_t type;     // PKT_SETTINGS
  uint8_t seq;
  uint8_t flags;
  uint8_t initialBattery;
  uint8_t responsiveness;
  int8_t  trim;
  uint8_t autoAltitude;
  uint8_t autoTaxiSpeed;
  uint8_t autoCircleSpeed;
  uint8_t desiredMode;
  uint8_t switch1;
  uint8_t switch2;
};
static_assert(sizeof(SettingsPacket) == 12, "SettingsPacket size");

// Plane ACK (4 bytes)
struct __attribute__((packed)) AckPacket {
  uint8_t type;     // PKT_ACK
  uint8_t ackSeq;   // echoes last received seq
  int8_t  rssi;     // dBm (approx)
  uint8_t status;   // bits: link OK etc.
};
static_assert(sizeof(AckPacket) == 4, "AckPacket size");

// Telemetry (12 bytes)
struct __attribute__((packed)) TelemetryPacket {
  uint8_t type;     // PKT_TELEMETRY
  uint8_t ackSeq;   // ACK coverage
  uint8_t mode;     // plane's current mode
  uint8_t battery;  // %
  int16_t alt_dm;   // altitude in decimeters
  uint16_t air_cmps;// airspeed in cm/s
  int16_t pitch_cd; // pitch in centi-deg
  int16_t roll_cd;  // roll in centi-deg
};
static_assert(sizeof(TelemetryPacket) == 12, "TelemetryPacket size");

// ============================== Globals for UI ===============================
bool g_radioLinkActive = false;
uint8_t g_desiredAirplaneMode = AIRPLANE_MODE_NEUTRAL;
bool g_controllerReceivesDataFromPlane = false;

uint8_t g_airplaneMode = AIRPLANE_MODE_NEUTRAL;
float   g_planeTelemetryBatteryPct  = 0.0f;
float   g_planeTelemetryHeightM     = 0.0f;
float   g_planeTelemetryAirspeedMps = 0.0f;

// Radio usage metrics
float g_radioTxPerSec = 0.0f;
float g_radioRxPerSec = 0.0f;
float g_radioTxTimeMs = 0.0f;

// ============================== Internal state ===============================
static volatile bool s_irqTxDone = false;
static volatile bool s_irqRxDone = false;

static volatile bool s_txBusy     = false;
static volatile uint32_t s_txStartUs = 0;
static volatile uint32_t s_lastTxDurUs = 3000;

static uint8_t  s_seq = 0;
static uint8_t  s_lastSentSeq = 255;
static uint8_t  s_lastAckSeq = 255;
static uint32_t s_nextSuperframeStartUs = 0;

static uint32_t s_ackDeadlineUs = 0;
static uint8_t  s_consecutiveMiss = 0;
static uint32_t s_lastMissResetMs = 0;

// Fixed bitrate (stability): 50 kbps
static constexpr uint8_t  s_rateIdx = 0;

// Duty-cycle bucket
static uint32_t s_dcCreditUs = kDcBucketCapUs / 2; // start half-full
static uint32_t s_dcLastRefillMs = 0;
static uint32_t s_toaEmaUs = 3000; // ToA EMA used for budgeting

// TX/RX counters for per-sec rates
static uint32_t s_isrTxCount = 0, s_isrRxCount = 0;

// Link supervision
static unsigned long s_lastPlaneFrameMs = 0;
static const unsigned long kConnectionTimeoutMs = 2000;

// Telemetry request state (sticky until sent)
static bool          s_tlmRequestPending = false;
static unsigned long s_lastTlmEventMs    = 0;   // last request SENT or telemetry RECEIVED

// Settings reliability state
static bool          s_settingsAwaitingAck = false;
static uint8_t       s_settingsSentSeq     = 0;
static unsigned long s_settingsRetryAtMs   = 0;
static bool          s_settingsNeedsResend = false;

// ------------------------------ ISR -----------------------------------------
#if defined(ESP32)
void IRAM_ATTR onDio1() {
#else
void onDio1() {
#endif
  if (s_txBusy) s_irqTxDone = true;
  else          s_irqRxDone = true;
}

// ------------------------------ PHY profiles --------------------------------
static void setFSKProfile(uint8_t rateIdx) {
  (void)rateIdx; // fixed 50 kbps for now
  radio.setPreambleLength(16);     // bytes
  radio.setCRC(true);
  radio.setDataShaping(0.5);       // Gaussian BT=0.5
  radio.setOutputPower(17);        // dBm
  radio.setBitRate(50.0);
  radio.setRxBandwidth(117.3);
  radio.setFrequencyDeviation(25.0);
}

// ------------------------------ DC governor ---------------------------------
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
static inline bool dcCanAfford(uint32_t estUs) { dcRefill(); return (s_dcCreditUs >= estUs); }
static inline void dcReserve(uint32_t estUs)   { if (estUs > s_dcCreditUs) estUs = s_dcCreditUs; s_dcCreditUs -= estUs; }

// ------------------------------ Helpers -------------------------------------
static inline uint32_t conservativeCostUs(uint8_t /*payloadLen*/) {
  uint32_t cost = (uint32_t)((float)s_toaEmaUs * kToASafety);
  if (cost < 2000) cost = 2000; // never under-reserve
  return cost;
}

static void scheduleNextFrame() {
  uint32_t nowUs = micros();
  if (s_nextSuperframeStartUs == 0) {
    s_nextSuperframeStartUs = nowUs + 2000; // start soon
  } else {
    s_nextSuperframeStartUs += kSuperframeUs;
    if ((int32_t)(s_nextSuperframeStartUs - nowUs) < 2000) {
      s_nextSuperframeStartUs = nowUs + 2000; // if we slipped, realign gently
    }
  }
}

static void beginRx() {
  goRx();
  delayMicroseconds(300);     // settle RF switch/paths (slightly longer)
  radio.startReceive();
}

// ------------------------------ TX logic ------------------------------------
static bool trySendControl(bool tlmReq) {
  if (s_txBusy) return false;

  ControlPacket p{};
  p.type     = PKT_CONTROL;
  p.seq      = s_seq;
  p.flags    = tlmReq ? FLG_TLM_REQ : 0;
  p.pitch    = (int8_t)lroundf(g_controlJoyRY * kJoyScale);
  p.roll     = (int8_t)lroundf(g_controlJoyRX * kJoyScale);
  p.throttle = (uint8_t)lroundf(g_controlKnob1 * kThrotScale);

  uint32_t cost = conservativeCostUs(sizeof(p));
  if (!dcCanAfford(cost)) return false;

  goTx();
  delayMicroseconds(300);
  s_txBusy = true;
  s_txStartUs = micros();
  int16_t st = radio.startTransmit((uint8_t*)&p, sizeof(p));
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

static bool trySendSettings() {
  if (s_txBusy) return false;

  SettingsPacket s{};
  s.type            = PKT_SETTINGS;
  s.seq             = s_seq;
  s.flags           = 0;
  s.initialBattery  = (uint8_t)getInitialBatteryPercent();
  s.responsiveness  = (uint8_t)getResponsiveness();
  s.trim            = (int8_t)getTrim();
  s.autoAltitude    = (uint8_t)getAutoAltitude();
  s.autoTaxiSpeed   = (uint8_t)getAutoTaxiSpeed();
  s.autoCircleSpeed = (uint8_t)getAutoCircleSpeed();
  s.desiredMode     = g_desiredAirplaneMode;
  s.switch1         = g_switch1;
  s.switch2         = g_switch2;

  uint32_t cost = conservativeCostUs(sizeof(s));
  if (!dcCanAfford(cost)) return false;

  goTx();
  delayMicroseconds(300);
  s_txBusy = true;
  s_txStartUs = micros();
  int16_t st = radio.startTransmit((uint8_t*)&s, sizeof(s));
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

// ------------------------------ RX handling ---------------------------------
static void handleRx() {
  uint8_t buf[32] = {0};
  size_t  len = sizeof(buf);
  int16_t st = radio.readData(buf, len);
  beginRx();
  if (st != RADIOLIB_ERR_NONE || len == 0) return;

  const uint8_t t = buf[0];
  s_isrRxCount++;

  if (t == PKT_ACK && len >= sizeof(AckPacket)) {
    const AckPacket* a = reinterpret_cast<const AckPacket*>(buf);
    s_lastAckSeq = a->ackSeq;
    s_consecutiveMiss = 0;
    s_lastMissResetMs = millis();
    g_controllerReceivesDataFromPlane = true;
    s_lastPlaneFrameMs = millis();
    if (s_settingsAwaitingAck && a->ackSeq == s_settingsSentSeq) {
      s_settingsAwaitingAck = false;
      s_settingsNeedsResend = false;
      s_settingsRetryAtMs = 0;
    }
  } else if (t == PKT_TELEMETRY && len >= sizeof(TelemetryPacket)) {
    const TelemetryPacket* tp = reinterpret_cast<const TelemetryPacket*>(buf);
    s_lastAckSeq = tp->ackSeq;
    s_consecutiveMiss = 0;
    s_lastMissResetMs = millis();
    g_controllerReceivesDataFromPlane = true;
    s_lastPlaneFrameMs = millis();
    if (s_settingsAwaitingAck && tp->ackSeq == s_settingsSentSeq) {
      s_settingsAwaitingAck = false;
      s_settingsNeedsResend = false;
      s_settingsRetryAtMs = 0;
    }

    // Mirror to UI
    g_airplaneMode               = tp->mode;
    g_planeTelemetryBatteryPct   = (float)tp->battery;
    g_planeTelemetryHeightM      = (float)tp->alt_dm * 0.1f;
    g_planeTelemetryAirspeedMps  = (float)tp->air_cmps * 0.01f;

    // We got telemetry: restart the 1 Hz interval
    s_lastTlmEventMs = millis();
    s_tlmRequestPending = false;
  }
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
  setFSKProfile(s_rateIdx);
  radio.setDio1Action(onDio1);

  beginRx();
  scheduleNextFrame();

  s_dcLastRefillMs = millis();
  s_lastMissResetMs = millis();
  s_lastPlaneFrameMs = millis();
  s_lastTlmEventMs = millis();
  g_radioLinkActive = true;
}

void handleCommunication() {
  // 1) IRQ service first
  if (s_irqTxDone) {
    s_irqTxDone = false;
    s_txBusy = false;
    uint32_t endUs = micros();
    s_lastTxDurUs = endUs - s_txStartUs;
    g_radioTxTimeMs = s_lastTxDurUs / 1000.0f;
    s_toaEmaUs = (uint32_t)(0.8f * s_toaEmaUs + 0.2f * (float)s_lastTxDurUs);
    s_isrTxCount++;
    beginRx();
    // ACK/Telemetry window deadline for this frame
    s_ackDeadlineUs = endUs + kGuardUs + kAckWindowUs;
  }
  if (s_irqRxDone) {
    s_irqRxDone = false;
    handleRx();
  }

  // 1b) TX hang watchdog
  if (s_txBusy) {
    uint32_t dt = micros() - s_txStartUs;
    if ((int32_t)dt > (int32_t)kTxHangTimeoutUs) {
      s_txBusy = false;
      radio.standby();
      delayMicroseconds(300);
      beginRx();
    }
  }

  // 2) Per-second TX/RX rate (simple EMA)
  static uint32_t lastTx = 0, lastRx = 0;
  static float txAvg = 0, rxAvg = 0;
  static unsigned long lastRateMs = millis();
  unsigned long nowMs = millis();
  if (nowMs - lastRateMs >= 1000) {
    uint32_t txD = s_isrTxCount - lastTx;
    uint32_t rxD = s_isrRxCount - lastRx;
    lastTx = s_isrTxCount; lastRx = s_isrRxCount; lastRateMs = nowMs;
    txAvg = 0.8f*txAvg + 0.2f*(float)txD;
    rxAvg = 0.8f*rxAvg + 0.2f*(float)rxD;
    g_radioTxPerSec = txAvg;
    g_radioRxPerSec = rxAvg;
  }

  // 3) Link supervision
  if ((nowMs - s_lastPlaneFrameMs) > kConnectionTimeoutMs) {
    g_controllerReceivesDataFromPlane = false;
  }

  // 4) Detect end of ack window -> mark miss (against last sent seq)
  uint32_t nowUs = micros();
  if (s_ackDeadlineUs && (int32_t)(nowUs - s_ackDeadlineUs) > 0) {
    if (s_lastAckSeq != s_lastSentSeq) {
      s_consecutiveMiss++;
      s_lastMissResetMs = nowMs;
    }
    s_ackDeadlineUs = 0;
  }

  // 4b) If settings packet wasn't acknowledged, schedule a retry after 1s
  if (s_settingsAwaitingAck && (long)(nowMs - s_settingsRetryAtMs) >= 0 &&
      s_lastAckSeq != s_settingsSentSeq) {
    s_settingsNeedsResend = true;
    s_settingsRetryAtMs = nowMs + 1000;
  }

  // 5) Make telemetry request sticky: once per >=1s, but keep pending until sent
  if (!s_tlmRequestPending &&
      g_controllerReceivesDataFromPlane &&
      (nowMs - s_lastTlmEventMs) >= kTelemetryIntervalMs) {
    s_tlmRequestPending = true;
  }

  // 6) Superframe timing: attempt exactly one send per frame
  if ((int32_t)(nowUs - s_nextSuperframeStartUs) >= 0) {
    bool sent = false;

    // Event-driven settings take priority
    static int s_prevInitBat=-1, s_prevResp=-1, s_prevTrim=0x7fffffff;
    static int s_prevAlt=-1, s_prevTaxi=-1, s_prevCirc=-1, s_prevMode=-1;
    static int s_prevSwitch1=-1, s_prevSwitch2=-1;

    int initBat = getInitialBatteryPercent();
    int resp    = getResponsiveness();
    int trim    = getTrim();
    int alt     = getAutoAltitude();
    int taxi    = getAutoTaxiSpeed();
    int circ    = getAutoCircleSpeed();
    int modeReq = (int)g_desiredAirplaneMode;
    int sw1     = g_switch1;
    int sw2     = g_switch2;

    bool settingsChanged =
      initBat!=s_prevInitBat || resp!=s_prevResp || trim!=s_prevTrim ||
      alt!=s_prevAlt || taxi!=s_prevTaxi || circ!=s_prevCirc || modeReq!=s_prevMode ||
      sw1!=s_prevSwitch1 || sw2!=s_prevSwitch2;

    if (settingsChanged || s_settingsNeedsResend) {
      sent = trySendSettings();
      if (sent) {
        s_lastSentSeq = s_seq;
        s_settingsSentSeq = s_seq;
        s_seq++;
        s_ackDeadlineUs = 0; // will be set on TX-done
        s_settingsAwaitingAck = true;
        s_settingsRetryAtMs = nowMs + 1000;
        s_settingsNeedsResend = false;
        if (settingsChanged) {
          s_prevInitBat=initBat; s_prevResp=resp; s_prevTrim=trim;
          s_prevAlt=alt; s_prevTaxi=taxi; s_prevCirc=circ; s_prevMode=modeReq;
          s_prevSwitch1=sw1; s_prevSwitch2=sw2;
        }
      }
    } else if (s_tlmRequestPending) {
      sent = trySendControl(true);
      if (sent) {
        s_lastSentSeq = s_seq;
        s_seq++;
        s_ackDeadlineUs = 0; // set on TX-done
        s_tlmRequestPending = false;     // request has actually been sent
        s_lastTlmEventMs = nowMs;        // start next 1s interval from here
      }
    } else {
      sent = trySendControl(false);
      if (sent) {
        s_lastSentSeq = s_seq;
        s_seq++;
        s_ackDeadlineUs = 0; // set on TX-done
      }
    }

    scheduleNextFrame();
  }
}
