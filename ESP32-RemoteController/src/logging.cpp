#include "logging.h"
#include "globals.h"
#include <Arduino.h>

namespace {
  bool logging_enabled = false;

  void tpValue(const char *name, double value) {
    Serial.print('>');
    Serial.print(name);
    Serial.print(':');
    Serial.println(value);
  }

  bool cond_logging_enabled() {
    while (Serial.available() > 0) {
      const int ch = Serial.read();
      if (ch == 't' || ch == 'T') {
        logging_enabled = !logging_enabled;
      }
    }
    return logging_enabled;
  }

  void job_log_dummy_counter() {
    tpValue("rc_dummy_counter", g_dummy_counter);
  }
}

void register_logging(JobRegistry& R) {
  /*Job                             Job type   Interval  last_run_ms  enabled  ready  condition             fn*/
  R.add({"logging_dummy_counter",  JOB_TIMER,  100,      0,           true,    false, cond_logging_enabled, job_log_dummy_counter});
}
