#include "logging.h"
#include "globals.h"
#include <Arduino.h>

namespace {
  bool logging_enabled = true;

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
    tpValue("g_counter1", g_counter1);
    tpValue("g_counter2", g_counter2);
    tpValue("cmd_mode", g_command_mode);
    tpValue("cmd_throttle", g_command_throttle);
    tpValue("cmd_roll", g_command_roll);
    tpValue("cmd_pitch", g_command_pitch);

    const uint32_t age_ms = millis() - g_command_last_update_ms;
    tpValue("cmd_age_ms", static_cast<double>(age_ms));
    tpValue("cmd_signal", g_command_has_signal ? 1.0 : 0.0);
    tpValue("cmd_rx_count", g_command_rx_count);
  }
}

void register_logging(JobRegistry& R) {
    /*Job                              Job type    Interval    last_run_ms   enabled   ready   condition               fn*/
    R.add({"logging_teleplot_globals", JOB_TIMER,  100,        0,            true,     false,  cond_logging_enabled,   job_teleplot_globals});
}
