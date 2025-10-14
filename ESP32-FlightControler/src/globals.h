#pragma once
#include <Arduino.h>

extern volatile uint32_t g_counter1;
extern volatile uint32_t g_counter2;

extern volatile uint8_t  g_command_mode;
extern volatile float    g_command_throttle;
extern volatile float    g_command_roll;
extern volatile float    g_command_pitch;
extern volatile uint32_t g_command_last_update_ms;
extern volatile bool     g_command_has_signal;
extern volatile uint32_t g_command_rx_count;
