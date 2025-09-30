#include "logging.h"
#include "globals.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <cstring>
#include <cstdio>

namespace {
  constexpr size_t   HISTORY_CAP = 64;        // ~12 seconds at 5 Hz
  constexpr size_t   PERSIST_QUEUE_CAP = 32;  // ~6 seconds at 5 Hz
  constexpr uint32_t ROTATION_INTERVAL_MS = 120000;  // 2 minutes
  constexpr size_t   HISTORY_FILES_TO_KEEP = 5;       // ~10 minutes of history
  constexpr size_t   MAX_DISCOVERED_FILES = 32;
  const char* const  LOG_FILE_PREFIX = "/global_history_";
  const char* const  LOG_FILE_SUFFIX = ".csv";

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
  bool     log_files_initialized = false;

  uint32_t current_file_index = 0;
  uint32_t current_file_start_ms = 0;
  bool     current_file_ready = false;
  char     current_file_path[32] = {0};

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

  String build_log_file_path(uint32_t index) {
    char buffer[32];
    snprintf(buffer,
             sizeof(buffer),
             "%s%06lu%s",
             LOG_FILE_PREFIX,
             static_cast<unsigned long>(index),
             LOG_FILE_SUFFIX);
    return String(buffer);
  }

  bool parse_log_file_index(const char* name, uint32_t& index_out) {
    const size_t prefix_len = strlen(LOG_FILE_PREFIX);
    const size_t suffix_len = strlen(LOG_FILE_SUFFIX);
    const size_t name_len = strlen(name);

    if (name_len <= prefix_len + suffix_len) {
      return false;
    }
    if (strncmp(name, LOG_FILE_PREFIX, prefix_len) != 0) {
      return false;
    }
    if (strncmp(name + name_len - suffix_len, LOG_FILE_SUFFIX, suffix_len) != 0) {
      return false;
    }

    const size_t digits_len = name_len - prefix_len - suffix_len;
    uint32_t     value = 0;
    for (size_t i = 0; i < digits_len; ++i) {
      const char ch = name[prefix_len + i];
      if (ch < '0' || ch > '9') {
        return false;
      }
      value = value * 10 + static_cast<uint32_t>(ch - '0');
    }

    index_out = value;
    return true;
  }

  void set_current_file(uint32_t index) {
    current_file_index = index;
    String path = build_log_file_path(index);
    strncpy(current_file_path, path.c_str(), sizeof(current_file_path));
    current_file_path[sizeof(current_file_path) - 1] = '\0';
    current_file_start_ms = millis();
    current_file_ready = true;
  }

  void enforce_retention() {
    if (!fs_ready) {
      return;
    }

    File root = LittleFS.open("/", FILE_READ);
    if (!root) {
      Serial.println(F("[logging] Failed to open filesystem root for retention"));
      return;
    }

    uint32_t indexes[MAX_DISCOVERED_FILES];
    size_t   count = 0;

    while (true) {
      File entry = root.openNextFile();
      if (!entry) {
        break;
      }

      uint32_t idx = 0;
      if (parse_log_file_index(entry.name(), idx)) {
        if (count < MAX_DISCOVERED_FILES) {
          indexes[count++] = idx;
        }
      }
      entry.close();
    }
    root.close();

    if (count <= HISTORY_FILES_TO_KEEP) {
      return;
    }

    // Simple selection sort to arrange indexes in ascending order.
    for (size_t i = 0; i + 1 < count; ++i) {
      size_t min_idx = i;
      for (size_t j = i + 1; j < count; ++j) {
        if (indexes[j] < indexes[min_idx]) {
          min_idx = j;
        }
      }
      if (min_idx != i) {
        uint32_t tmp = indexes[i];
        indexes[i] = indexes[min_idx];
        indexes[min_idx] = tmp;
      }
    }

    const size_t to_delete = count - HISTORY_FILES_TO_KEEP;
    for (size_t i = 0; i < to_delete; ++i) {
      String path = build_log_file_path(indexes[i]);
      if (!LittleFS.remove(path.c_str())) {
        Serial.print(F("[logging] Failed to remove old log file: "));
        Serial.println(path);
      }
    }
  }

  void rotate_log_file() {
    uint32_t next_index = current_file_ready ? (current_file_index + 1) : 0;
    set_current_file(next_index);

    File f = LittleFS.open(current_file_path, FILE_WRITE);
    if (!f) {
      Serial.println(F("[logging] Failed to create rotated log file"));
      current_file_ready = false;
      return;
    }
    write_header_if_empty(f);
    f.close();
    enforce_retention();
  }

  void initialize_log_files() {
    if (log_files_initialized || !fs_ready) {
      return;
    }

    File root = LittleFS.open("/", FILE_READ);
    if (!root) {
      Serial.println(F("[logging] Failed to scan filesystem for logs"));
      return;
    }

    bool     found_any = false;
    uint32_t max_index = 0;

    while (true) {
      File entry = root.openNextFile();
      if (!entry) {
        break;
      }

      uint32_t idx = 0;
      if (parse_log_file_index(entry.name(), idx)) {
        if (!found_any || idx > max_index) {
          max_index = idx;
          found_any = true;
        }
      }
      entry.close();
    }
    root.close();

    if (found_any) {
      set_current_file(max_index);
      File f = LittleFS.open(current_file_path, FILE_APPEND);
      if (f) {
        write_header_if_empty(f);
        f.close();
      } else {
        Serial.println(F("[logging] Failed to open existing log file"));
        current_file_ready = false;
      }
    } else {
      rotate_log_file();
    }

    log_files_initialized = current_file_ready;
  }

  void ensure_log_file() {
    if (!fs_ready) {
      return;
    }

    if (!log_files_initialized) {
      initialize_log_files();
    }

    if (!current_file_ready) {
      rotate_log_file();
      log_files_initialized = current_file_ready;
    }
  }

  void flush_pending_to_fs() {
    if (persist_size == 0) return;
    ensure_filesystem();
    if (!fs_ready) return;

    ensure_log_file();
    if (!current_file_ready) {
      return;
    }

    if (millis() - current_file_start_ms >= ROTATION_INTERVAL_MS) {
      rotate_log_file();
      if (!current_file_ready) {
        return;
      }
    }

    File f = LittleFS.open(current_file_path, FILE_APPEND);
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

    File root = LittleFS.open("/", FILE_READ);
    if (!root) {
      Serial.println(F("[logging] Failed to open filesystem root for clear"));
      return;
    }

    bool any_failure = false;
    while (true) {
      File entry = root.openNextFile();
      if (!entry) {
        break;
      }

      uint32_t idx = 0;
      if (parse_log_file_index(entry.name(), idx)) {
        String path = build_log_file_path(idx);
        if (!LittleFS.remove(path.c_str())) {
          Serial.print(F("[logging] Failed to remove log file during clear: "));
          Serial.println(path);
          any_failure = true;
        }
      }
      entry.close();
    }
    root.close();

    if (!any_failure) {
      current_file_ready = false;
      log_files_initialized = false;
    }
  }

  bool dump_persistent_history() {
    ensure_filesystem();
    if (!fs_ready) {
      Serial.println(F("[logging] LittleFS unavailable"));
      return false;
    }

    File root = LittleFS.open("/", FILE_READ);
    if (!root) {
      Serial.println(F("[logging] Failed to open filesystem root for dump"));
      return false;
    }

    uint32_t indexes[MAX_DISCOVERED_FILES];
    size_t   count = 0;

    while (true) {
      File entry = root.openNextFile();
      if (!entry) {
        break;
      }

      uint32_t idx = 0;
      if (parse_log_file_index(entry.name(), idx)) {
        if (count < MAX_DISCOVERED_FILES) {
          indexes[count++] = idx;
        }
      }
      entry.close();
    }
    root.close();

    if (count == 0) {
      Serial.println(F("[logging] No persisted history yet"));
      return false;
    }

    for (size_t i = 0; i + 1 < count; ++i) {
      size_t min_idx = i;
      for (size_t j = i + 1; j < count; ++j) {
        if (indexes[j] < indexes[min_idx]) {
          min_idx = j;
        }
      }
      if (min_idx != i) {
        uint32_t tmp = indexes[i];
        indexes[i] = indexes[min_idx];
        indexes[min_idx] = tmp;
      }
    }

    bool data_written = false;
    for (size_t i = 0; i < count; ++i) {
      String path = build_log_file_path(indexes[i]);
      File   f = LittleFS.open(path.c_str(), FILE_READ);
      if (!f) {
        Serial.print(F("[logging] Failed to read log file: "));
        Serial.println(path);
        continue;
      }

      bool header_skipped = false;
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
    }

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
