#include "comms_jobs.h"
#include "globals.h"
#include <Arduino.h>
#include <HardwareSerial.h>

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

static bool cond_always_on() {
  return true;
}

static HardwareSerial& pi_uart() {
  static bool initialized = false;
  static HardwareSerial& serial = Serial2;
  if (!initialized) {
    serial.begin(115200, SERIAL_8N1, 16, 17);
    initialized = true;
  }
  return serial;
}

struct TelemetryItem {
  const uint8_t* data;
  size_t size;
};

static void job_uart_send_globals() {
  const TelemetryItem items[] = {
      {reinterpret_cast<const uint8_t*>(&g_counter1), sizeof(g_counter1)},
      {reinterpret_cast<const uint8_t*>(&g_counter2), sizeof(g_counter2)},
  };

  HardwareSerial& serial = pi_uart();
  const uint8_t start_indicator = 0xAA;
  serial.write(start_indicator);

  for (const TelemetryItem& item : items) {
    serial.write(item.data, item.size);
  }
}

void register_comms_jobs(JobRegistry& R) {
  /*Job                     Job type   Interval  last_run_ms  enabled  ready  condition                         fn*/
  R.add({"comms_on_c1_x10", JOB_EVENT, 0,        0,           true,    false, cond_counter1_multiple_of_10_once, job_increment_on_c1x10});
  R.add({"comms_uart_globals", JOB_TIMER, 200,      0,           true,    false, cond_always_on,                   job_uart_send_globals});
}
