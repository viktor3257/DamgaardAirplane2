#include <Arduino.h>
#include "job_registry.h"
#include "globals.h"
#include "control_jobs.h"
#include "comms_jobs.h"

static JobRegistry REG;
static uint32_t lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait if needed */ }

  register_control_jobs(REG);
  register_comms_jobs(REG);

  Serial.print("Jobs registered: ");
  Serial.println(REG.size());
}

void loop() {
  const uint32_t now = millis();

  // Phase 1: evaluate readiness for all jobs (both timer and event)
  REG.evaluate(now);

  // Phase 2: execute ready jobs
  REG.execute(now);

  // Optional: show it's working once per second
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.print("counter1 = "); Serial.print(g_counter1);
    Serial.print(" | counter2 = "); Serial.println(g_counter2);
  }
}
