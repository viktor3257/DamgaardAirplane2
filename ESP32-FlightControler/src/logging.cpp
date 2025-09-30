#include "logging.h"
#include "globals.h"
#include <Arduino.h>

namespace {
  bool logging_enabled = false;

  static void tpValue(const char *name, double value) {
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

  void job_teleplot_globals() {
    tpValue("tp:g_counter1", g_counter1);
    tpValue("tp:g_counter2", g_counter2);
  }
}

void register_logging(JobRegistry& R) {
  Job log_job = {
    "logging_teleplot_globals",
    JOB_TIMER,
    /*interval_ms=*/100,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_logging_enabled,
    /*fn=*/job_teleplot_globals
  };
  R.add(log_job);
}
