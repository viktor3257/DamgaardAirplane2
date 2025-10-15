#include "serial_debug.h"
#include "communication.h"
#include "sensors.h"
#include "navigation.h"
#include "control.h"

void initSerialDebug(unsigned long baud) {
    Serial.begin(baud);
    Serial.println(">clear|cmd");
}

template <typename T>
static void tpValue(const char *name, T value) {
    Serial.print('>');
    Serial.print(name);
    Serial.print(':');
    Serial.println(value);
}

// Overload for doubles to allow printing with higher precision. Used for
// latitude/longitude values that require more decimal places.
static void tpValue(const char *name, double value) {
    Serial.print('>');
    Serial.print(name);
    Serial.print(':');
    Serial.println(value, 6);
}


void printPlaneDebugInfo() {
    static unsigned long lastPrintTime = 0;
    const unsigned long kIntervalMs = 100; // limit Teleplot output rate
    unsigned long now = millis();
    if (now - lastPrintTime < kIntervalMs) {
        return;
    }
    lastPrintTime = now;

    tpValue("Plane_Mode", g_airplaneMode);
    tpValue("Plane_AltitudeM", g_airplaneHeight);
    tpValue("Plane_HeadingDeg", g_airplaneHeading);
    tpValue("Plane_AirSpeedMps", g_airplaneAirSpeed);
    tpValue("Plane_GroundSpeedMps", g_airplaneGroundSpeed);
    tpValue("Plane_ClimbRateMps", g_airplaneClimbRate);
    tpValue("Plane_PitchDeg", g_airplanePitch);
    tpValue("Plane_RollDeg", g_airplaneRoll);
    tpValue("Plane_DesiredPitch", g_desiredPitch);
    tpValue("Plane_DesiredRoll", g_desiredRoll);
    tpValue("Plane_BatteryPercent", g_airplaneBatteryPercent);
    tpValue("Plane_CurrentA", g_airplaneCurrentA);
    tpValue("Plane_CurrentSensorV", g_airplaneCurrentSensorV);

    tpValue("Plane_Latitude", g_airplaneLatitude);
    tpValue("Plane_Longitude", g_airplaneLongitude);
    tpValue("Plane_TargetLatitude", g_circleTargetLatitude);
    tpValue("Plane_TargetLongitude", g_circleTargetLongitude);

    tpValue("Plane_RecController", g_airplaneReceivesDataFromController);
    tpValue("Plane_RecPi", g_airplaneReceivesDataFromRaspberryPi);
    tpValue("Plane_RouteAgeS", g_areaSearchRouteAge);
    tpValue("Plane_RadioTxPerSec", g_radioTxPerSec);
    tpValue("Plane_RadioRxPerSec", g_radioRxPerSec);

    // Values received from the controller
    tpValue("manualRoll", g_controlJoyRX);
    tpValue("manualPitch", g_controlJoyRY);
    tpValue("Plane_Knob1", g_controlKnob1);
    tpValue("Plane_Knob2", g_controlKnob2);
    tpValue("Plane_Knob3", g_controlKnob3);

    tpValue("Plane_InitialBatteryPct", g_settingInitialBatteryPercent);
    tpValue("Plane_Responsiveness", g_settingResponsiveness);
    tpValue("Plane_Trim", g_settingTrim);
    tpValue("Plane_AutoAltitude", g_settingAutoAltitude);
    tpValue("Plane_AutoTaxiSpeed", g_settingAutoTaxiSpeed);
    tpValue("Plane_AutoCircleSpeed", g_settingAutoCircleSpeed);

    tpValue("Plane_ThrottleOutput", g_outputThrottle);
    tpValue("Plane_LoopTimeMs", g_loopTimeMs);

    tpValue("Plane_DesiredHeading", g_desiredHeading);
    tpValue("Plane_DesiredAltitude", g_desiredAltitude);
    tpValue("Plane_DesiredSpeed", g_desiredSpeed);
}
