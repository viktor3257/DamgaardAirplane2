#include "control_jobs.h"
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

bool cond_always_true() { return true; }

int mapJoystick(int raw, int minVal, int midVal, int maxVal, bool invert) {
  int out;
  if (raw <= midVal) {
    out = map(raw, minVal, midVal, 0, 50);
  } else {
    out = map(raw, midVal, maxVal, 50, 99);
  }
  out = constrain(out, 0, 99);
  return invert ? (99 - out) : out;
}

int calibrateAxis(int pin, int samples = 20) {
  long sum = 0;
  for (int i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(5);
  }
  return static_cast<int>(sum / samples);
}

void calibrateJoysticks() {
  g_j1x_mid = calibrateAxis(kJoy1XPin);
  g_j1y_mid = calibrateAxis(kJoy1YPin);
}

void initButtons(uint32_t now_ms) {
  for (auto& btn : g_buttons) {
    pinMode(btn.pin, INPUT_PULLUP);
    const bool pressed = (digitalRead(btn.pin) == LOW);
    btn.stable_pressed  = pressed;
    btn.last_sample     = pressed;
    btn.last_change_ms  = now_ms;
    *btn.output         = pressed ? 1 : 0;
  }
}

void initSwitches() {
  pinMode(kSwitch1Pin, INPUT_PULLUP);
  pinMode(kSwitch2Pin, INPUT_PULLUP);
  g_switch1_state = (digitalRead(kSwitch1Pin) == LOW) ? 1 : 0;
  g_switch2_state = (digitalRead(kSwitch2Pin) == LOW) ? 1 : 0;
}

void updateButton(DebouncedButton& btn, uint32_t now_ms) {
  const bool sample_pressed = (digitalRead(btn.pin) == LOW);
  if (sample_pressed != btn.last_sample) {
    btn.last_sample    = sample_pressed;
    btn.last_change_ms = now_ms;
  }

  if ((now_ms - btn.last_change_ms) >= kDebounceMs &&
      sample_pressed != btn.stable_pressed) {
    btn.stable_pressed = sample_pressed;
  }

  *btn.output = btn.stable_pressed ? 1 : 0;
}

void job_read_buttons_and_switches() {
  const uint32_t now = millis();
  for (auto& btn : g_buttons) {
    updateButton(btn, now);
  }

  g_switch1_state = (digitalRead(kSwitch1Pin) == LOW) ? 1 : 0;
  g_switch2_state = (digitalRead(kSwitch2Pin) == LOW) ? 1 : 0;
}

void job_read_potentiometers() {
  g_pot1_raw = analogRead(kPot1Pin);
  g_pot2_raw = analogRead(kPot2Pin);

  const int pot1_val = map(g_pot1_raw, 0, 4095, 0, 99);
  const int pot2_val = map(g_pot2_raw, 0, 4095, 0, 99);

  g_control_knob1 = static_cast<float>(pot1_val) / 99.0f;
  g_control_knob2 = static_cast<float>(pot2_val) / 99.0f;
}

void job_read_joystick() {
  g_joy1x_raw = analogRead(kJoy1XPin);
  g_joy1y_raw = analogRead(kJoy1YPin);

  const int jx_val = mapJoystick(g_joy1x_raw, J1X_MIN, g_j1x_mid, J1X_MAX, false);
  const int jy_val = mapJoystick(g_joy1y_raw, J1Y_MIN, g_j1y_mid, J1Y_MAX, false);

  g_control_joy_rx = static_cast<float>(jx_val) / 99.0f;
  g_control_joy_ry = static_cast<float>(jy_val) / 99.0f;
}

void initialise_inputs() {
  const uint32_t now_ms = millis();
  initButtons(now_ms);
  initSwitches();
  calibrateJoysticks();

  job_read_potentiometers();
  job_read_joystick();
}
} // namespace

void register_control_jobs(JobRegistry& R) {
  initialise_inputs();

  R.add({
      "ctl_buttons_switches",
      JOB_TIMER,
      kButtonPollIntervalMs,
      0,
      true,
      false,
      cond_always_true,
      job_read_buttons_and_switches,
  });

  R.add({
      "ctl_potentiometers",
      JOB_TIMER,
      kPotPollIntervalMs,
      0,
      true,
      false,
      cond_always_true,
      job_read_potentiometers,
  });

  R.add({
      "ctl_joystick",
      JOB_TIMER,
      kJoyPollIntervalMs,
      0,
      true,
      false,
      cond_always_true,
      job_read_joystick,
  });
}
