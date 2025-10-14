#include "comms_jobs.h"
#include "globals.h"

#include <RadioLib.h>

namespace {
  constexpr uint32_t kRadioJobIntervalMs = 100;  // 10 Hz broadcast rate

  constexpr float   kRadioFrequencyMHz   = 869.525f;  // EU868 O-band
  constexpr float   kRadioBandwidthKHz   = 250.0f;
  constexpr uint8_t kRadioSpreading      = 5;
  constexpr uint8_t kRadioCodingRate     = 5;         // 4/5 coding rate
  constexpr uint8_t kRadioSyncWord       = RADIOLIB_SX126X_SYNC_WORD_PUBLIC;
  constexpr int8_t  kRadioTxPowerDbm     = 10;

  constexpr uint8_t kRadioCsPin          = 5;
  constexpr uint8_t kRadioDio1Pin        = 27;
  constexpr uint8_t kRadioResetPin       = 26;
  constexpr uint8_t kRadioBusyPin        = 25;
  constexpr uint8_t kRadioRxEnPin        = 15;

  Module g_radio_module(kRadioCsPin, kRadioDio1Pin, kRadioResetPin, kRadioBusyPin);
  SX1262 g_radio(&g_radio_module);

  bool g_radio_initialized = false;

  bool ensure_radio_initialized() {
    if (g_radio_initialized) {
      return true;
    }

    g_radio_module.setRfSwitchPins(kRadioRxEnPin, kRadioRxEnPin);

    const int16_t state = g_radio.begin(kRadioFrequencyMHz,
                                        kRadioBandwidthKHz,
                                        kRadioSpreading,
                                        kRadioCodingRate,
                                        kRadioSyncWord,
                                        kRadioTxPowerDbm);
    if (state != RADIOLIB_ERR_NONE) {
      return false;
    }

    g_radio.setDio2AsRfSwitch(true);

    g_radio_initialized = true;
    return true;
  }

  struct __attribute__((packed)) ControlPacket {
    char    magic;
    uint8_t mode;
    float   throttle;
    float   roll;
    float   pitch;
  };

  void send_control_packet() {
    if (!ensure_radio_initialized()) {
      return;
    }

    ControlPacket packet{};
    packet.magic    = 'R';
    packet.mode     = static_cast<uint8_t>(g_controller_mode);
    packet.throttle = g_control_knob1;
    packet.roll     = g_control_joy_rx;
    packet.pitch    = g_control_joy_ry;

    g_radio.transmit(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  }

  bool cond_always_true() { return true; }

  void job_increment_dummy_counter() {
    g_dummy_counter += 1;
  }
}  // namespace

void register_comms_jobs(JobRegistry& R) {
  /*Job                          Job type   Interval              last_run_ms  enabled  ready  condition           fn*/
  R.add({"comms_dummy_counter",   JOB_TIMER,  200,                 0,           true,    false, cond_always_true, job_increment_dummy_counter});
  R.add({"comms_radio_broadcast", JOB_TIMER,  kRadioJobIntervalMs, 0,           true,    false, cond_always_true, send_control_packet});
}
