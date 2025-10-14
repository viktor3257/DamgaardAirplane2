#include "sensor_jobs.h"
#include "globals.h"
#include <Arduino.h>

namespace {
constexpr int kButton1Pin = 13;
constexpr int kButton2Pin = 14;
constexpr int kButton3Pin = 33;
constexpr int kButton4Pin = 32;

constexpr int kPot1Pin = 36;
constexpr int kPot2Pin = 39;

constexpr int kSwitch1Pin = 16;
constexpr int kSwitch2Pin = 17;

constexpr int kJoy1XPin = 34;
constexpr int kJoy1YPin = 35;

constexpr uint32_t kButtonPollIntervalMs = 10;
constexpr uint32_t kSwitchPollIntervalMs = 10;
constexpr uint32_t kPotPollIntervalMs    = 20;
constexpr uint32_t kJoyPollIntervalMs    = 20;
constexpr uint32_t kDebounceMs           = 40;

// Estimated joystick limits based on observed extremes
constexpr int J1X_MIN = 490;
constexpr int J1X_MAX = 3150;
constexpr int J1Y_MIN = 530;
constexpr int J1Y_MAX = 3590;

struct DebouncedButton {
  uint8_t  pin;
  uint8_t* output;
  bool     stable_pressed;
  bool     last_sample;
  uint32_t last_change_ms;
};

static DebouncedButton g_buttons[] = {
    {static_cast<uint8_t>(kButton1Pin), &g_button1_state, false, false, 0},
    {static_cast<uint8_t>(kButton2Pin), &g_button2_state, false, false, 0},
    {static_cast<uint8_t>(kButton3Pin), &g_button3_state, false, false, 0},
    {static_cast<uint8_t>(kButton4Pin), &g_button4_state, false, false, 0},
};

static int g_j1x_mid = 2048;
static int g_j1y_mid = 2048;

void job_buttons();
void job_potentiometers();
void job_switches();
void job_joystick();

void init_buttons(uint32_t now_ms) {
  for (auto& btn : g_buttons) {
    pinMode(btn.pin, INPUT_PULLUP);
    const bool pressed = (digitalRead(btn.pin) == LOW);
    btn.stable_pressed  = pressed;
    btn.last_sample     = pressed;
    btn.last_change_ms  = now_ms;
    *btn.output         = pressed ? 1 : 0;
  }
}

void init_potentiometers() {
  pinMode(kPot1Pin, INPUT);
  pinMode(kPot2Pin, INPUT);
  job_potentiometers();
}

void init_switches() {
  pinMode(kSwitch1Pin, INPUT_PULLUP);
  pinMode(kSwitch2Pin, INPUT_PULLUP);
  job_switches();
}

void init_joystick() {
  pinMode(kJoy1XPin, INPUT);
  pinMode(kJoy1YPin, INPUT);

  long sum_x = 0;
  long sum_y = 0;
  constexpr int kSamples = 20;
  for (int i = 0; i < kSamples; ++i) {
    sum_x += analogRead(kJoy1XPin);
    sum_y += analogRead(kJoy1YPin);
    delay(5);
  }
  g_j1x_mid = static_cast<int>(sum_x / kSamples);
  g_j1y_mid = static_cast<int>(sum_y / kSamples);

  job_joystick();
}

void job_buttons() {
  const uint32_t now = millis();
  for (auto& btn : g_buttons) {
    const bool sample_pressed = (digitalRead(btn.pin) == LOW);
    if (sample_pressed != btn.last_sample) {
      btn.last_sample    = sample_pressed;
      btn.last_change_ms = now;
    }

    if ((now - btn.last_change_ms) >= kDebounceMs) {
      btn.stable_pressed = sample_pressed;
    }

    *btn.output = btn.stable_pressed ? 1 : 0;
  }
}

void job_potentiometers() {
  g_pot1_raw = analogRead(kPot1Pin);
  g_pot2_raw = analogRead(kPot2Pin);

  const int pot1_val = map(g_pot1_raw, 0, 4095, 0, 99);
  const int pot2_val = map(g_pot2_raw, 0, 4095, 0, 99);

  g_control_knob1 = static_cast<float>(pot1_val) / 99.0f;
  g_control_knob2 = static_cast<float>(pot2_val) / 99.0f;
}

void job_switches() {
  g_switch1_state = (digitalRead(kSwitch1Pin) == LOW) ? 1 : 0;
  g_switch2_state = (digitalRead(kSwitch2Pin) == LOW) ? 1 : 0;
}

void job_joystick() {
  g_joy1x_raw = analogRead(kJoy1XPin);
  g_joy1y_raw = analogRead(kJoy1YPin);

  int jx_val;
  if (g_joy1x_raw <= g_j1x_mid) {
    jx_val = map(g_joy1x_raw, J1X_MIN, g_j1x_mid, 0, 50);
  } else {
    jx_val = map(g_joy1x_raw, g_j1x_mid, J1X_MAX, 50, 99);
  }
  int jy_val;
  if (g_joy1y_raw <= g_j1y_mid) {
    jy_val = map(g_joy1y_raw, J1Y_MIN, g_j1y_mid, 0, 50);
  } else {
    jy_val = map(g_joy1y_raw, g_j1y_mid, J1Y_MAX, 50, 99);
  }

  jx_val = constrain(jx_val, 0, 99);
  jy_val = constrain(jy_val, 0, 99);

  g_control_joy_rx = static_cast<float>(jx_val) / 99.0f;
  g_control_joy_ry = static_cast<float>(jy_val) / 99.0f;
}

} // namespace

void register_sensor_jobs(JobRegistry& R) {
  const uint32_t now_ms = millis();
  init_buttons(now_ms);
  init_potentiometers();
  init_switches();
  init_joystick();

  R.add({
      "ctl_buttons",
      JOB_TIMER,
      kButtonPollIntervalMs,
      0,
      true,
      false,
      nullptr,
      job_buttons,
  });

  R.add({
      "ctl_switches",
      JOB_TIMER,
      kSwitchPollIntervalMs,
      0,
      true,
      false,
      nullptr,
      job_switches,
  });

  R.add({
      "ctl_potentiometers",
      JOB_TIMER,
      kPotPollIntervalMs,
      0,
      true,
      false,
      nullptr,
      job_potentiometers,
  });

  R.add({
      "ctl_joystick",
      JOB_TIMER,
      kJoyPollIntervalMs,
      0,
      true,
      false,
      nullptr,
      job_joystick,
  });
}
