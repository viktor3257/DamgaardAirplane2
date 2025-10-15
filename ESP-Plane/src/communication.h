#pragma once

// Communication between controller and plane
#include <stdint.h>

enum AirplaneMode {
    AIRPLANE_MODE_NEUTRAL = 0,
    AIRPLANE_MODE_MANUAL = 1,
    AIRPLANE_MODE_CIRCLE = 2,
    AIRPLANE_MODE_AREA_SEARCH = 3
};

enum PacketType {
    PACKET_TYPE_MANUAL = 1,
    PACKET_TYPE_SETTINGS = 2,
};

void initCommunication();
void handleCommunication();


extern bool g_radioLinkActive;
extern uint8_t g_airplaneMode;

extern float g_controlKnob1;
extern float g_controlKnob2;
extern float g_controlKnob3;
extern float g_controlJoyRX;
extern float g_controlJoyRY;

extern bool g_airplaneReceivesDataFromController;
extern float g_radioTxPerSec;
extern float g_radioRxPerSec;

// Controller settings sent to the plane
extern int g_settingInitialBatteryPercent;
extern int g_settingResponsiveness;
extern int g_settingTrim;
extern int g_settingAutoAltitude;
extern int g_settingAutoTaxiSpeed;
extern int g_settingAutoCircleSpeed;
