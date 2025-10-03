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

static bool cond_always_on() { return true; }

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
  /*Job                         Job type  Interval  last_run_ms  enabled  ready  condition                          fn*/
  R.add({"comms_on_c1_x10",     JOB_EVENT, 0,        0,           true,    false, cond_counter1_multiple_of_10_once, job_increment_on_c1x10});
  R.add({"comms_uart_globals",  JOB_TIMER,  200,     0,           true,    false, cond_always_on,                    job_uart_send_globals_csv});
}
