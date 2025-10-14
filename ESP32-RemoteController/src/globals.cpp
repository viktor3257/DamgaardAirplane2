#include "globals.h"

volatile uint32_t g_dummy_counter = 0;

float   g_control_knob1 = 0.0f;
float   g_control_knob2 = 0.0f;
float   g_control_joy_rx = 0.0f;
float   g_control_joy_ry = 0.0f;

uint8_t g_switch1_state = 0;
uint8_t g_switch2_state = 0;

uint8_t g_button1_state = 0;
uint8_t g_button2_state = 0;
uint8_t g_button3_state = 0;
uint8_t g_button4_state = 0;

int     g_pot1_raw = 0;
int     g_pot2_raw = 0;
int     g_joy1x_raw = 0;
int     g_joy1y_raw = 0;
