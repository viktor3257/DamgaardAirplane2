#pragma once
#include "job_model.h"

// Keep it dead-simple: fixed capacity, no templates required.
#define MAX_JOBS 32

class JobRegistry {
public:
  JobRegistry() : n_(0) {}

  bool add(const Job& j) {
    if (n_ >= MAX_JOBS) return false;
    jobs_[n_] = j;
    // Basic init for new entries
    jobs_[n_].ready = false;
    if (jobs_[n_].kind == JOB_TIMER) {
      jobs_[n_].last_run_ms = millis(); // start counting from now
    }
    n_++;
    return true;
  }

  void evaluate(uint32_t now_ms) {
    for (size_t i = 0; i < n_; ++i) {
      Job& j = jobs_[i];
      if (!j.enabled) continue;

      if (j.kind == JOB_TIMER) {
        if ((now_ms - j.last_run_ms) >= j.interval_ms) {
          if (!j.condition || j.condition()) {
            j.ready = true;
          }
          // If condition is false, leave not-ready and don't update last_run_ms.
        }
      } else { // JOB_EVENT
        if (j.condition && j.condition()) {
          j.ready = true; // edge behavior belongs in the condition itself
        }
      }
    }
  }

  void execute(uint32_t now_ms) {
    for (size_t i = 0; i < n_; ++i) {
      Job& j = jobs_[i];
      if (!j.ready) continue;

      j.fn();
      if (j.kind == JOB_TIMER) {
        j.last_run_ms = now_ms;
      }
      j.ready = false;
    }
  }

  size_t size() const { return n_; }

private:
  Job     jobs_[MAX_JOBS];
  size_t  n_;
};
