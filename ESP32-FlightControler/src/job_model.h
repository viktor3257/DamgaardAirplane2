#pragma once
#include <Arduino.h>

enum JobKind : uint8_t { JOB_TIMER, JOB_EVENT };

struct Job {
  const char* name;
  JobKind     kind;

  // TIMER fields (ignored for EVENT)
  uint32_t    interval_ms;
  uint32_t    last_run_ms;

  // Common
  bool        enabled;
  bool        ready;          // set by evaluate(), cleared by execute()
  bool      (*condition)();   // for TIMER: nullptr => treated as true
  void      (*fn)();
};
