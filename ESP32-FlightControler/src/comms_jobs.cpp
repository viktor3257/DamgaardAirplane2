#include "comms_jobs.h"
#include "globals.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <RadioLib.h>

// Trigger once when counter1 hits a non-zero multiple of 10
static bool cond_counter1_multiple_of_10_once() {
  static uint32_t last_triggered_on = 0;
  const uint32_t c = g_counter1;
  if (c != 0 && (c % 10 == 0) && (c != last_triggered_on)) {
    last_triggered_on = c;
    return true;
  }
  return false;
}

static void job_increment_on_c1x10() {
  g_counter2 += 1;
}

static bool cond_always_on() { return true; }

namespace {
  constexpr uint32_t kRadioPollIntervalMs = 20;

  constexpr float   kRadioFrequencyMHz   = 869.525f;
  constexpr float   kRadioBandwidthKHz   = 250.0f;
  constexpr uint8_t kRadioSpreading      = 5;
  constexpr uint8_t kRadioCodingRate     = 5;
  constexpr uint8_t kRadioSyncWord       = RADIOLIB_SX126X_SYNC_WORD_PUBLIC;

  constexpr int8_t  kRadioTxPowerDbm     = 10;

  constexpr uint8_t kRadioCsPin          = 5;
  constexpr uint8_t kRadioDio1Pin        = 34;
  constexpr uint8_t kRadioResetPin       = 2;
  constexpr uint8_t kRadioBusyPin        = 35;
  constexpr uint8_t kRadioRxEnPin        = 4;

  Module g_radio_module(kRadioCsPin, kRadioDio1Pin, kRadioResetPin, kRadioBusyPin);
  SX1262 g_radio(&g_radio_module);

  bool g_radio_initialized = false;

  struct __attribute__((packed)) ControlPacket {
    char    magic;
    uint8_t mode;
    float   throttle;
    float   roll;
    float   pitch;
  };

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
    g_radio.startReceive();

    g_radio_initialized = true;
    return true;
  }

  bool decode_control_packet(const ControlPacket& packet) {
    if (packet.magic != 'R') {
      return false;
    }

    g_command_mode           = packet.mode;
    g_command_throttle       = packet.throttle;
    g_command_roll           = packet.roll;
    g_command_pitch          = packet.pitch;
    g_command_last_update_ms = millis();
    g_command_has_signal     = true;
    g_command_rx_count      += 1;

    return true;
  }

  void job_radio_receive_control() {
    if (!ensure_radio_initialized()) {
      g_command_has_signal = false;
      return;
    }

    ControlPacket packet{};
    const int16_t state = g_radio.readData(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    if (state == RADIOLIB_ERR_NONE) {
      if (!decode_control_packet(packet)) {
        g_command_has_signal = false;
      }
    } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
      g_command_has_signal = false;
    }

    g_radio.startReceive();
  }
} // namespace

// Lazy-inited UART (adjust pins: begin(baud, config, RX, TX))
static HardwareSerial& pi_uart() {
  static bool initialized = false;
  static HardwareSerial& serial = Serial2;
  if (!initialized) {
    serial.begin(115200, SERIAL_8N1, 16, 17);
    initialized = true;
  }
  return serial;
}

// ---- Minimal CSV: "g_counter1,g_counter2\n" (no snapshots) ----
static void job_uart_send_globals_csv() {
  char line[48];
  // Use the volatile globals directly in the format string
  const int n = snprintf(line, sizeof(line), "%lu,%lu\n",
                         (unsigned long)g_counter1,
                         (unsigned long)g_counter2);
  if (n > 0) {
    pi_uart().print(line);
  }
}

void register_comms_jobs(JobRegistry& R) {
  /*Job                         Job type  Interval          last_run_ms  enabled  ready  condition                          fn*/
  R.add({"comms_radio_receive", JOB_TIMER,  kRadioPollIntervalMs, 0,
         true,    false, cond_always_on,                    job_radio_receive_control});
  R.add({"comms_on_c1_x10",     JOB_EVENT,  0,                   0,
         true,    false, cond_counter1_multiple_of_10_once, job_increment_on_c1x10});
  R.add({"comms_uart_globals",  JOB_TIMER,  200,                 0,
         true,    false, cond_always_on,                    job_uart_send_globals_csv});
}
