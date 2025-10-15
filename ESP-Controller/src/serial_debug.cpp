#include "serial_debug.h"
#include "communication.h"
#include "sensors.h"
#include "display.h"

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

void printControllerDebugInfo() {
    static unsigned long lastPrintTime = 0;
    const unsigned long kIntervalMs = 100; // limit Teleplot output rate
    unsigned long now = millis();
    if (now - lastPrintTime < kIntervalMs) {
        return;
    }
    lastPrintTime = now;

    tpValue("Controller_pack_Throttle", g_controlKnob1);
    tpValue("Controller_pack_MenuKnob", g_controlKnob2);
    tpValue("Controller_pack_manualRoll", g_controlJoyRX);
    tpValue("Controller_pack_manualPitch", g_controlJoyRY);
    tpValue("Controller_Mode", g_airplaneMode);
    tpValue("Controller_pack_DesiredMode", g_desiredAirplaneMode);
    tpValue("Controller_pack_Switch1", g_switch1);
    tpValue("Controller_pack_Switch2", g_switch2);
    tpValue("Controller_RecPlane", g_controllerReceivesDataFromPlane);

    tpValue("Controller_LoopTimeMs", g_loopTimeMs);
    tpValue("Controller_RadioTxTimeMs", g_radioTxTimeMs);
    tpValue("Controller_RadioTxPerSec", g_radioTxPerSec);
    tpValue("Controller_RadioRxPerSec", g_radioRxPerSec);
    tpValue("Controller_pack_InitialBatteryPct", getInitialBatteryPercent());
    tpValue("Controller_pack_Responsiveness", getResponsiveness());
    tpValue("Controller_pack_Trim", getTrim());
    tpValue("Controller_pack_AutoAltitude", getAutoAltitude());
    tpValue("Controller_pack_AutoTaxiSpeed", getAutoTaxiSpeed());
    tpValue("Controller_pack_AutoCircleSpeed", getAutoCircleSpeed());
}
