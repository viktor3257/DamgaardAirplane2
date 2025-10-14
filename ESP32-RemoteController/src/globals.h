#pragma once
#include <Arduino.h>

extern volatile uint32_t g_dummy_counter;

extern float   g_control_knob1;
extern float   g_control_knob2;
extern float   g_control_joy_rx;
extern float   g_control_joy_ry;

extern uint8_t g_switch1_state;
extern uint8_t g_switch2_state;

extern uint8_t g_button1_state;
extern uint8_t g_button2_state;
extern uint8_t g_button3_state;
extern uint8_t g_button4_state;

extern int     g_pot1_raw;
extern int     g_pot2_raw;
extern int     g_joy1x_raw;
extern int     g_joy1y_raw;

enum ControllerMode : uint8_t {
  CONTROLLER_MODE_NEUTRAL = 0,
  CONTROLLER_MODE_MANUAL  = 1,
  CONTROLLER_MODE_CIRCLING = 2,
};

struct ControllerSettings {
  int initial_battery_percent;
  int responsiveness_percent;
  int trim_degrees;
  int auto_altitude_meters;
  int auto_taxi_speed_kph;
  int auto_circle_speed_kph;
};

extern ControllerMode      g_controller_mode;
extern ControllerSettings  g_controller_settings;
