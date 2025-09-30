#include "logging.h"
#include "globals.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

namespace {
  constexpr size_t HISTORY_CAP = 64;        // ~12 seconds at 5 Hz
  constexpr size_t PERSIST_QUEUE_CAP = 32;  // ~6 seconds at 5 Hz
  const char* const LOG_FILE_PATH = "/global_history.csv";

  struct LogEntry {
    uint32_t timestamp_ms;
    uint32_t counter1;
    uint32_t counter2;
  };

  // RAM history ring buffer (keeps most recent HISTORY_CAP samples)
  LogEntry history_buffer[HISTORY_CAP];
  size_t   history_head = 0;  // Next position to write
  size_t   history_size = 0;  // Number of valid entries in buffer

  // Pending entries waiting to be persisted
  LogEntry persist_queue[PERSIST_QUEUE_CAP];
  size_t   persist_size = 0;

  bool     logging_enabled = false;
  bool     dump_requested = false;
  bool     fs_ready = false;

  void ensure_filesystem() {
    if (!fs_ready) {
      fs_ready = LittleFS.begin(true);
      if (!fs_ready) {
        Serial.println(F("[logging] Failed to mount LittleFS"));
      }
    }
  }

  void flush_pending_to_fs();

  void capture_globals_job() {
    const LogEntry entry{millis(), g_counter1, g_counter2};

    history_buffer[history_head] = entry;
    history_head = (history_head + 1) % HISTORY_CAP;
    if (history_size < HISTORY_CAP) {
      history_size++;
    }

    if (persist_size >= PERSIST_QUEUE_CAP) {
      Serial.println(F("[logging] Persist queue full, forcing flush"));
      flush_pending_to_fs();
    }

    if (persist_size >= PERSIST_QUEUE_CAP) {
      Serial.println(F("[logging] Dropping oldest pending entry"));
      for (size_t i = 1; i < persist_size; ++i) {
        persist_queue[i - 1] = persist_queue[i];
      }
      persist_size = PERSIST_QUEUE_CAP - 1;
    }

    persist_queue[persist_size++] = entry;
  }

  void write_header_if_empty(File& f) {
    if (f.size() == 0) {
      f.println(F("timestamp_ms,counter1,counter2"));
    }
  }

  void flush_pending_to_fs() {
    if (persist_size == 0) return;
    ensure_filesystem();
    if (!fs_ready) return;

    File f = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
    if (!f) {
      Serial.println(F("[logging] Failed to open history file"));
      return;
    }

    write_header_if_empty(f);

    for (size_t i = 0; i < persist_size; ++i) {
      const LogEntry& e = persist_queue[i];
      f.printf("%lu,%lu,%lu\n",
               static_cast<unsigned long>(e.timestamp_ms),
               static_cast<unsigned long>(e.counter1),
               static_cast<unsigned long>(e.counter2));
    }
    f.close();
    persist_size = 0;
  }

  static void tpValue(const char *name, double value) {
    Serial.print('>');
    Serial.print(name);
    Serial.print(':');
    Serial.println(value);
  }

  bool cond_logging_enabled() {
    return logging_enabled;
  }

  void job_teleplot_globals() {
    tpValue("g_counter1", g_counter1);
    tpValue("g_counter2", g_counter2);
  }

  bool cond_persist_needed() {
    return persist_size > 0;
  }

  void clear_persistent_history() {
    ensure_filesystem();
    if (!fs_ready) {
      return;
    }

    if (!LittleFS.exists(LOG_FILE_PATH)) {
      return;
    }

    if (!LittleFS.remove(LOG_FILE_PATH)) {
      Serial.println(F("[logging] Failed to clear history file"));
    }
  }

  bool dump_persistent_history() {
    ensure_filesystem();
    if (!fs_ready) {
      Serial.println(F("[logging] LittleFS unavailable"));
      return false;
    }

    if (!LittleFS.exists(LOG_FILE_PATH)) {
      Serial.println(F("[logging] No persisted history yet"));
      return false;
    }

    File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
    if (!f) {
      Serial.println(F("[logging] Failed to read history file"));
      return false;
    }

    bool header_skipped = false;
    bool data_written = false;
    while (f.available()) {
      int c = f.read();
      if (!header_skipped) {
        if (c == '\n') {
          header_skipped = true;
        }
        continue;
      }
      Serial.write(c);
      data_written = true;
    }
    f.close();
    return data_written;
  }

  bool cond_dump_requested() {
    while (Serial.available() > 0) {
      const int ch = Serial.read();
      if (ch == 't' || ch == 'T') {
        logging_enabled = !logging_enabled;
        Serial.print(F("[logging] Teleplot logging "));
        Serial.println(logging_enabled ? F("enabled") : F("disabled"));
      } else if (ch == 'l' || ch == 'L') {
        dump_requested = true;
      }
    }

    return dump_requested;
  }

  void job_dump_history() {
    dump_requested = false;
    flush_pending_to_fs();
    if (dump_persistent_history()) {
      clear_persistent_history();
    }
  }
}

void register_logging(JobRegistry& R) {
  ensure_filesystem();

  Job log_job = {
    "logging_teleplot_globals",
    JOB_TIMER,
    /*interval_ms=*/100,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_logging_enabled,
    /*fn=*/job_teleplot_globals
  };
  R.add(log_job);

  Job capture_job = {
    "logging_capture_globals",
    JOB_TIMER,
    /*interval_ms=*/200,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/nullptr,
    /*fn=*/capture_globals_job
  };
  R.add(capture_job);

  Job persist_job = {
    "logging_persist_history",
    JOB_TIMER,
    /*interval_ms=*/5000,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_persist_needed,
    /*fn=*/flush_pending_to_fs
  };
  R.add(persist_job);

  Job dump_job = {
    "logging_dump_history",
    JOB_EVENT,
    /*interval_ms=*/0,
    /*last_run_ms=*/0,
    /*enabled=*/true,
    /*ready=*/false,
    /*condition=*/cond_dump_requested,
    /*fn=*/job_dump_history
  };
  R.add(dump_job);
}
