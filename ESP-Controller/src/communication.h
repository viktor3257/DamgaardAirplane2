#pragma once

// Communication between controller and plane
#include <stdint.h>

enum AirplaneMode {
    AIRPLANE_MODE_NEUTRAL = 0,
    AIRPLANE_MODE_MANUAL = 1,
    AIRPLANE_MODE_CIRCLE = 2
};

enum PacketType {
    PACKET_TYPE_MANUAL = 1,
    PACKET_TYPE_SETTINGS = 2,
};

void initCommunication();
void handleCommunication();

extern bool g_radioLinkActive;
extern uint8_t g_airplaneMode;
extern uint8_t g_desiredAirplaneMode;

extern float g_controlKnob1;
extern float g_controlKnob2;
extern float g_controlJoyRX;
extern float g_controlJoyRY;

// Radio communication diagnostics
extern float g_radioTxTimeMs;
extern float g_radioTxPerSec;
extern float g_radioRxPerSec;

// Plane telemetry values mirrored on the controller
extern float g_planeTelemetryBatteryPct;
extern float g_planeTelemetryHeightM;
extern float g_planeTelemetryAirspeedMps;

extern bool g_controllerReceivesDataFromPlane;
