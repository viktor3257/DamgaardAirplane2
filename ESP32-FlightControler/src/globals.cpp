#include "globals.h"

volatile uint32_t g_counter1 = 0; // control loop counter
volatile uint32_t g_counter2 = 0; // comms/event counter

volatile uint8_t  g_command_mode            = 0;
volatile float    g_command_throttle        = 0.0f;
volatile float    g_command_roll            = 0.0f;
volatile float    g_command_pitch           = 0.0f;
volatile uint32_t g_command_last_update_ms  = 0;
volatile bool     g_command_has_signal      = false;
volatile uint32_t g_command_rx_count        = 0;
