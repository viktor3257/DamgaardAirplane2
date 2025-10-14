#include "display_jobs.h"
#include "globals.h"

#include <Arduino.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

namespace {
constexpr uint32_t kDisplayIntervalMs = 50;

hd44780_I2Cexp lcd;
bool            g_display_ready = false;

struct SettingDescriptor {
  const char* name_short;
  const char* name_full;
  const char* unit;
  int         min_value;
  int         max_value;
  int*        value_ptr;
};

SettingDescriptor g_settings[] = {
    {"Bat%", "Initial battery%", "%", 1,   100, &g_controller_settings.initial_battery_percent},
    {"Resp", "Responsiveness (manual)", "%", 1,   100, &g_controller_settings.responsiveness_percent},
    {"Trim", "Trim (manual)", "deg", -10, 10,  &g_controller_settings.trim_degrees},
    {"AltH", "Auto. Altitude", "m", 0,   130, &g_controller_settings.auto_altitude_meters},
    {"Taxi", "Auto. Taxi speed", "km/t", 35, 60, &g_controller_settings.auto_taxi_speed_kph},
    {"Circ", "Auto. Circling speed", "km/t", 35, 60, &g_controller_settings.auto_circle_speed_kph},
};

constexpr size_t kNumSettings = sizeof(g_settings) / sizeof(g_settings[0]);

int  g_selected_index = 0;
int  g_top_index      = 0;
bool g_edit_mode      = false;
int  g_prev_value     = 0;
int  g_edit_value     = 0;

char g_last_lines[4][21];

uint8_t g_prev_button_states[4] = {0, 0, 0, 0};

const char* mode_name(ControllerMode mode) {
  switch (mode) {
    case CONTROLLER_MODE_NEUTRAL:
      return "Neutral";
    case CONTROLLER_MODE_MANUAL:
      return "Manual";
    case CONTROLLER_MODE_CIRCLING:
      return "Circling";
    default:
      return "Unknown";
  }
}

void reset_display_cache() {
  std::memset(g_last_lines, 0, sizeof(g_last_lines));
}

bool init_display() {
  Wire.begin(21, 22);
  Wire.setClock(100000);
  const int status = lcd.begin(20, 4);
  if (status != 0) {
    Serial.print("LCD begin error: ");
    Serial.println(status);
    return false;
  }
  lcd.clear();
  reset_display_cache();
  return true;
}

void update_line(int row, const char* text) {
  if (row < 0 || row >= 4) {
    return;
  }

  char padded[21];
  int  len = std::strlen(text);
  if (len > 20) {
    len = 20;
  }

  for (int i = 0; i < 20; ++i) {
    padded[i] = (i < len) ? text[i] : ' ';
  }
  padded[20] = '\0';

  char* prev = g_last_lines[row];
  for (int col = 0; col < 20;) {
    if (padded[col] != prev[col]) {
      int start = col;
      while (col < 20 && padded[col] != prev[col]) {
        ++col;
      }
      lcd.setCursor(start, row);
      for (int c = start; c < col; ++c) {
        lcd.print(padded[c]);
        prev[c] = padded[c];
      }
    } else {
      ++col;
    }
  }
}

void clear_display_cache() {
  if (!g_display_ready) {
    return;
  }
  lcd.clear();
  reset_display_cache();
}

void print_edit_screen(const SettingDescriptor& setting) {
  char line[21];

  std::snprintf(line, sizeof(line), "%s (%s)", setting.name_full, setting.unit);
  update_line(0, line);

  std::snprintf(line, sizeof(line), "Range %d-%d %s", setting.min_value, setting.max_value, setting.unit);
  update_line(1, line);

  std::snprintf(line, sizeof(line), "Prev:%4d %s", g_prev_value, setting.unit);
  update_line(2, line);

  std::snprintf(line, sizeof(line), "New: %4d %s", g_edit_value, setting.unit);
  update_line(3, line);
}

void print_main_screen() {
  for (int row = 0; row < 4; ++row) {
    char line[21];
    std::memset(line, ' ', sizeof(line));

    const int idx = g_top_index + row;
    if (idx < static_cast<int>(kNumSettings)) {
      const SettingDescriptor& setting = g_settings[idx];
      const int               value   = *setting.value_ptr;
      const char               marker  = (idx == g_selected_index) ? '>' : ' ';

      char left[11];
      std::snprintf(left, sizeof(left), "%c%-5s%4d", marker, setting.name_short, value);
      const size_t left_len = std::strlen(left);
      const size_t copy_len = (left_len > 10) ? 10 : left_len;
      std::memcpy(line, left, copy_len);
    }

    if (row == 0) {
      char right[11];
      std::snprintf(right, sizeof(right), "Mode:%s", mode_name(g_controller_mode));
      const size_t right_len = std::strlen(right);
      const size_t copy_len  = (right_len > 9) ? 9 : right_len;
      std::memcpy(line + 11, right, copy_len);
    } else if (row == 1) {
      std::memcpy(line + 11, "B1:Ntrl", 7);
    } else if (row == 2) {
      std::memcpy(line + 11, "B2:Manl", 7);
    } else if (row == 3) {
      std::memcpy(line + 11, "B3:Circ", 7);
    }

    line[20] = '\0';
    update_line(row, line);
  }
}

void handle_button_edges() {
  const uint8_t states[4] = {
      g_button1_state,
      g_button2_state,
      g_button3_state,
      g_button4_state,
  };

  if (states[0] && !g_prev_button_states[0]) {
    g_controller_mode = CONTROLLER_MODE_NEUTRAL;
  }
  if (states[1] && !g_prev_button_states[1]) {
    g_controller_mode = CONTROLLER_MODE_MANUAL;
  }
  if (states[2] && !g_prev_button_states[2]) {
    g_controller_mode = CONTROLLER_MODE_CIRCLING;
  }

  const bool select_pressed = states[3] && !g_prev_button_states[3];
  for (size_t i = 0; i < 4; ++i) {
    g_prev_button_states[i] = states[i];
  }

  if (!g_display_ready) {
    return;
  }

  if (select_pressed) {
    SettingDescriptor& setting = g_settings[g_selected_index];
    if (g_edit_mode) {
      *setting.value_ptr = g_edit_value;
    } else {
      g_prev_value = *setting.value_ptr;
      g_edit_value = g_prev_value;
    }
    g_edit_mode = !g_edit_mode;
    clear_display_cache();
  }
}

void update_display_job() {
  handle_button_edges();

  if (!g_display_ready) {
    return;
  }

  if (g_edit_mode) {
    SettingDescriptor& setting = g_settings[g_selected_index];
    const float       knob     = g_control_knob2;
    const int         min_val  = setting.min_value;
    const int         max_val  = setting.max_value;
    const int         range    = max_val - min_val;

    int new_value = min_val + static_cast<int>(knob * range + 0.5f);
    if (new_value < min_val) {
      new_value = min_val;
    }
    if (new_value > max_val) {
      new_value = max_val;
    }

    g_edit_value = new_value;
    print_edit_screen(setting);
  } else {
    int new_index = static_cast<int>(g_control_knob2 * static_cast<float>(kNumSettings));
    if (new_index < 0) {
      new_index = 0;
    }
    if (new_index >= static_cast<int>(kNumSettings)) {
      new_index = static_cast<int>(kNumSettings) - 1;
    }

    if (new_index != g_selected_index) {
      g_selected_index = new_index;
      if (g_selected_index < g_top_index) {
        g_top_index = g_selected_index;
      } else if (g_selected_index > g_top_index + 3) {
        g_top_index = g_selected_index - 3;
      }
    }

    print_main_screen();
  }
}

}  // namespace

void register_display_jobs(JobRegistry& R) {
  if (!g_display_ready) {
    g_display_ready = init_display();
  }

  R.add({"ui_display", JOB_TIMER, kDisplayIntervalMs, 0, true, false, nullptr, update_display_job});
}
