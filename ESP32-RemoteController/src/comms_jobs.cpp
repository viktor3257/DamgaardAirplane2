#include "comms_jobs.h"
#include "globals.h"

namespace {
  bool cond_always_true() { return true; }

  void job_increment_dummy_counter() {
    g_dummy_counter += 1;
  }
}

void register_comms_jobs(JobRegistry& R) {
  /*Job                          Job type   Interval  last_run_ms  enabled  ready  condition           fn*/
  R.add({"comms_dummy_counter",  JOB_TIMER,  200,      0,           true,    false, cond_always_true, job_increment_dummy_counter});
}
