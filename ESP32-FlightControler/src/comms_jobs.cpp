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
  Job evt = {
    "comms_on_c1_x10",
    JOB_EVENT,
    /*interval_ms=*/0,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_counter1_multiple_of_10_once,
    /*fn=*/job_increment_on_c1x10
  };
  R.add(evt);
}
