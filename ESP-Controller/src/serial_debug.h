#pragma once
#include <Arduino.h>

void initSerialDebug(unsigned long baud = 921600);
void printControllerDebugInfo();

// Time between iterations of the main loop in milliseconds. Updated in
// main.cpp and published through the Teleplot output.
extern float g_loopTimeMs;
