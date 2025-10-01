#include "control_jobs.h"
#include "globals.h"

// ----- Conditions -----
// For now: always true. Later: gate by mode/arming/etc.
static bool cond_always_true() { return true; }

// ----- Implementations -----
static void job_increment_every_100ms() {
  g_counter1 += 1;
}

// ----- Registration -----
void register_control_jobs(JobRegistry& R) {
  /*Job                       Job type    Interval    last_run_ms   enabled   ready   condition             fn*/
  R.add({"ctrl_every_100ms",  JOB_TIMER,  100,        0,            true,     false,  cond_always_true,     job_increment_every_100ms});
}
