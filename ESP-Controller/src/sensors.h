#pragma once

#include <stdint.h>

void initSensors();
void readSensors();
// Update the servo output based on current sensor state
void updateServo();

// Returns true if button 4 has been pressed since the last call.
bool button4Pressed();

extern float g_controlKnob1;
extern float g_controlKnob2;
extern float g_controlJoyRX;
extern float g_controlJoyRY;

extern uint8_t g_switch1;
extern uint8_t g_switch2;
