#include "comms_jobs.h"
#include "globals.h"

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

void register_comms_jobs(JobRegistry& R) {
  /*Job                     Job type   Interval  last_run_ms  enabled  ready  condition                         fn*/
  R.add({"comms_on_c1_x10", JOB_EVENT, 0,        0,           true,    false, cond_counter1_multiple_of_10_once, job_increment_on_c1x10});
}
