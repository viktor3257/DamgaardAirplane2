#pragma once

void initSensors();
void readSensors();

#include <stdint.h>

extern float g_airplaneHeight;
extern float g_airplaneHeading;
extern float g_airplaneAirSpeed;
extern float g_airplaneGroundSpeed;
extern float g_airplaneClimbRate;
extern float g_airplanePitch;
extern float g_airplaneRoll;
extern float g_airplaneBatteryPercent;
extern float g_airplaneCurrentA;
extern float g_airplaneCurrentSensorV;

extern double g_airplaneLatitude;
extern double g_airplaneLongitude;
extern double g_airplaneHomeLatitude;
extern double g_airplaneHomeLongitude;
extern double g_circleTargetLatitude;
extern double g_circleTargetLongitude;

extern bool g_airplaneReceivesDataFromRaspberryPi;
