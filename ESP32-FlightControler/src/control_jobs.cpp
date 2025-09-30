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
  Job every100 = {
    "ctrl_every_100ms",
    JOB_TIMER,
    /*interval_ms=*/100,
    /*last_run_ms=*/0, // registry will set it to now
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_always_true,
    /*fn=*/job_increment_every_100ms
  };
  R.add(every100);
}
