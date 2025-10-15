#include <Arduino.h>
#include "communication.h"
#include "sensors.h"
#include "display.h"
#include "serial_debug.h"

// Time between consecutive loop iterations in milliseconds
float g_loopTimeMs = 0.0f;
static unsigned long g_lastLoopMicros = 0;


void setup() {
    initSerialDebug();
    initCommunication();
    initSensors();
    initDisplay();
}

void loop() {
    unsigned long now = micros();
    g_loopTimeMs = (now - g_lastLoopMicros) / 1000.0f;
    g_lastLoopMicros = now;

    readSensors();
    updateServo();
    handleCommunication();
    updateDisplay();
    printControllerDebugInfo();
    delay(1); // Small delay to avoid flooding the serial output
}
