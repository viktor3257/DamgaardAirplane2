// Minimal table-driven scheduler (Hz-based) + logging + Teleplot toggle (T) + Log dump (L)
// ESP32 + Arduino core (PlatformIO). Open Serial @115200, NO line ending.
//
// Jobs:
//   - CTRL: increments `counter` (example workload)
//   - TELEPLOT: streams ">counter:VALUE" when enabled by 'T'
//   - LOG_SAMPLE: 4 Hz -> buffer in RAM
//   - LOG_FLUSH:  every 5 s -> append CSV to LittleFS
//   - SERIAL_KEYS: handles 'T' and 'L'

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

// ---------------- App state ----------------
volatile uint32_t counter = 0;
bool teleplot_on = false;                // toggled by 'T'
const char* LOG_FILE = "/counter_log.csv";

// RAM log buffer
struct Sample { uint32_t t_ms; uint32_t val; };
Sample logbuf[256];
size_t loglen = 0;
bool log_header_written = false;

// ---------------- Job functions ----------------
void job_ctrl() { counter++; }

void job_teleplot() {
  if (!teleplot_on) return;
  // Teleplot format: >name:value  (value with 6 decimals)
  Serial.print('>');
  Serial.print("counter");
  Serial.print(':');
  Serial.println((double)counter, 6);
}

void job_log_sample() {
  if (loglen < 256) logbuf[loglen++] = Sample{ millis(), counter };
}

void job_log_flush() {
  if (loglen == 0) return;
  if (!LittleFS.begin(true)) { Serial.println("[LOG] LittleFS init FAILED"); return; }

  // First flush in session writes/overwrites header
  const char* mode = log_header_written ? "a" : "w";
  File f = LittleFS.open(LOG_FILE, mode);
  if (!f) { Serial.println("[LOG] Open FAILED"); return; }
  if (!log_header_written) { f.println("t_ms,counter"); log_header_written = true; }

  for (size_t i=0; i<loglen; ++i) {
    f.print(logbuf[i].t_ms); f.print(','); f.println(logbuf[i].val);
  }
  f.close();
  loglen = 0;
}

void job_serial_keys() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c=='t' || c=='T') {
      teleplot_on = !teleplot_on;
    } else if (c=='l' || c=='L') {
      if (!LittleFS.begin(true) || !LittleFS.exists(LOG_FILE)) {
        Serial.println("[LOG] No file"); continue;
      }
      File f = LittleFS.open(LOG_FILE, "r");
      if (!f) { Serial.println("[LOG] Open read FAILED"); continue; }
      Serial.print("BEGIN_LOG "); Serial.println(LOG_FILE);
      while (f.available()) Serial.write(f.read());
      f.close();
      Serial.println("END_LOG");
    }
  }
}

// ---------------- Tiny scheduler (Hz table) ----------------
typedef void (*JobFn)();
struct Job { const char* name; float hz; JobFn fn; uint32_t period_us; uint32_t due_us; };

Job jobs[] = {
  { "CTRL",         200.0f, job_ctrl,        0, 0 },
  { "SERIAL_KEYS",  100.0f, job_serial_keys, 0, 0 },
  { "TELEPLOT",      20.0f, job_teleplot,    0, 0 },
  { "LOG_SAMPLE",     4.0f, job_log_sample,  0, 0 },
  { "LOG_FLUSH",      0.2f, job_log_flush,   0, 0 }, // every 5 s
};
const size_t N = sizeof(jobs)/sizeof(jobs[0]);

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  LittleFS.begin(true); // ok if already formatted/mounted

  uint32_t now = micros();
  for (size_t i=0; i<N; ++i) {
    jobs[i].period_us = (uint32_t)(1000000.0f / jobs[i].hz + 0.5f);
    jobs[i].due_us    = now + jobs[i].period_us; // first run after one period
  }
}

void loop() {
  uint32_t now = micros();
  for (size_t i=0; i<N; ++i) {
    if ((int32_t)(now - jobs[i].due_us) >= 0) {
      jobs[i].fn();
      jobs[i].due_us += jobs[i].period_us;                 // drift-free
      while ((int32_t)(now - jobs[i].due_us) >= 0)         // catch-up
        jobs[i].due_us += jobs[i].period_us;
    }
  }
}
