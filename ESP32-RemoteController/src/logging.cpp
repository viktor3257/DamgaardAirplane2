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

  void job_log_potentiometers() {
    tpValue("rc_pot1_raw", g_pot1_raw);
    tpValue("rc_pot2_raw", g_pot2_raw);
    tpValue("rc_pot1", g_control_knob1);
    tpValue("rc_pot2", g_control_knob2);
  }

  void job_log_switches() {
    tpValue("rc_switch1", g_switch1_state);
    tpValue("rc_switch2", g_switch2_state);
  }

  void job_log_joystick() {
    tpValue("rc_joy_x_raw", g_joy1x_raw);
    tpValue("rc_joy_y_raw", g_joy1y_raw);
    tpValue("rc_joy_rx", g_control_joy_rx);
    tpValue("rc_joy_ry", g_control_joy_ry);
  }
}

void register_logging(JobRegistry& R) {
  /*Job                             Job type   Interval  last_run_ms  enabled  ready  condition             fn*/
  R.add({"logging_dummy_counter",  JOB_TIMER,  100,      0,           true,    false, cond_logging_enabled, job_log_dummy_counter});
  R.add({"logging_potentiometers", JOB_TIMER,  100,      0,           true,    false, cond_logging_enabled, job_log_potentiometers});
  R.add({"logging_switches",       JOB_TIMER,  100,      0,           true,    false, cond_logging_enabled, job_log_switches});
  R.add({"logging_joystick",       JOB_TIMER,  100,      0,           true,    false, cond_logging_enabled, job_log_joystick});
}
