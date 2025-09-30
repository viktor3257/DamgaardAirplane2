#include <Arduino.h>
#include "job_registry.h"
#include "control_jobs.h"
#include "comms_jobs.h"
#include "logging.h"

static JobRegistry REG;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait if needed */ }

  register_control_jobs(REG);
  register_comms_jobs(REG);
  register_logging(REG);

  Serial.print("Jobs registered: ");
  Serial.println(REG.size());
}

void loop() {
  const uint32_t now = millis();

  REG.evaluate(now);
  REG.execute(now);
}
