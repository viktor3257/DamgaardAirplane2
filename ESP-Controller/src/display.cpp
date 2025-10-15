#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <cstring>
#include "display.h"
#include "communication.h"
#include "sensors.h"

// 20x4 I2C LCD using auto-detected backpack
static hd44780_I2Cexp lcd; // auto-detect PCF8574 mapping

static constexpr int kNumSettings = 6;

static const char *g_settingNames[kNumSettings] = {
    "Bat%", "Resp", "Trim", "AltH", "Taxi", "Circ"
};
static const char *g_settingFullNames[kNumSettings] = {
    "Initial battery%", "Responsiveness (manual)", "Trim (manual)",
    "Auto. Altitude", "Auto. Taxi speed", "Auto. Circling speed"
};

static const int g_settingMin[kNumSettings] = {1, 1, -10, 0, 35, 35};
static const int g_settingMax[kNumSettings] = {100, 100, 10, 130, 60, 60};
static const char *g_settingUnits[kNumSettings] = {
    "%", "%", "deg", "m", "km/t", "km/t"
};

static int g_settingValues[kNumSettings] = {100, 50, 3, 50, 40, 40};
static int g_selectedIndex = 0;
static int g_topIndex = 0;
static bool g_editMode = false;
static int g_prevValue = 0;
static int g_editValue = 0;

static const char* g_modeNames[] = {
    "Neutral", "Manual", "Circle", "ASearch", "Failsaf"};

// Previously printed lines, used to avoid unnecessary updates
static char g_lastLines[4][21] = {{0}};

// Update a line on the LCD only where characters differ
static void updateLine(int row, const char *text) {
    char padded[21];
    int len = strlen(text);
    if (len > 20) len = 20;
    for (int i = 0; i < 20; ++i) {
        padded[i] = (i < len) ? text[i] : ' ';
    }
    padded[20] = '\0';

    char *prev = g_lastLines[row];
    for (int col = 0; col < 20;) {
        if (padded[col] != prev[col]) {
            int start = col;
            while (col < 20 && padded[col] != prev[col]) ++col;
            lcd.setCursor(start, row);
            for (int c = start; c < col; ++c) {
                lcd.print(padded[c]);
                prev[c] = padded[c];
            }
        } else {
            ++col;
        }
    }
}
static void printLinePadded(int row, const char *text) { updateLine(row, text); }

int getInitialBatteryPercent() { return g_settingValues[0]; }
int getResponsiveness() { return g_settingValues[1]; }
int getTrim() { return g_settingValues[2]; }
int getAutoAltitude() { return g_settingValues[3]; }
int getAutoTaxiSpeed() { return g_settingValues[4]; }
int getAutoCircleSpeed() { return g_settingValues[5]; }

void initDisplay() {
    Wire.begin(21, 22);
    Wire.setClock(100000); // 100 kHz is safest
    int status = lcd.begin(20, 4);
    if (status) {
        Serial.print("LCD begin error: ");
        Serial.println(status);
        for (;;)
            ;
    }
    lcd.clear();
    memset(g_lastLines, 0, sizeof(g_lastLines));
}

static void printMainScreen() {
    char line[21];
    for (int row = 0; row < 4; ++row) {
        memset(line, ' ', 20);
        int idx = g_topIndex + row;
        if (idx < kNumSettings) {
            char left[11];
            char marker = (idx == g_selectedIndex) ? '>' : ' ';
            snprintf(left, sizeof(left), "%c%-5s%4d", marker, g_settingNames[idx],
                     g_settingValues[idx]);
            int len = strlen(left);
            if (len > 10) len = 10;
            memcpy(line, left, len);
        }

        if (!g_controllerReceivesDataFromPlane) {
            static const char *noTelem[4] = {
                "+-------+",
                "| NO    |",
                "| DATA! |",
                "+-------+"
            };
            memcpy(line + 11, noTelem[row], 9);
        } else if (row == 0) {
            const char *mode = "Unknown";
            if (g_airplaneMode < sizeof(g_modeNames) / sizeof(g_modeNames[0])) {
                mode = g_modeNames[g_airplaneMode];
            }
            char right[10];
            snprintf(right, sizeof(right), "M:%s", mode);
            int len = strlen(right);
            if (len > 9) len = 9;
            memcpy(line + 11, right, len);
        } else if (row == 1) {
            char right[10];
            snprintf(right, sizeof(right), "B:%3.0f%%", g_planeTelemetryBatteryPct);
            int len = strlen(right);
            if (len > 9) len = 9;
            memcpy(line + 11, right, len);
        } else if (row == 2) {
            char right[10];
            snprintf(right, sizeof(right), "H:%4.0f", g_planeTelemetryHeightM);
            int len = strlen(right);
            if (len > 9) len = 9;
            memcpy(line + 11, right, len);
        } else if (row == 3) {
            char right[10];
            snprintf(right, sizeof(right), "S:%4.0f", g_planeTelemetryAirspeedMps);
            int len = strlen(right);
            if (len > 9) len = 9;
            memcpy(line + 11, right, len);
        }
        line[20] = '\0';
        updateLine(row, line);
    }
}

void updateDisplay() {
    static int lastSwitch2State = -1;
    int switch2State = g_switch2 ? 1 : 0;

    if (lastSwitch2State == -1) {
        lastSwitch2State = switch2State;
        if (switch2State) {
            g_prevValue = g_settingValues[g_selectedIndex];
            g_editValue = g_prevValue;
            g_editMode  = true;
        }
    } else if (switch2State != lastSwitch2State) {
        if (switch2State) {
            g_prevValue = g_settingValues[g_selectedIndex];
            g_editValue = g_prevValue;
            g_editMode  = true;
        } else {
            if (g_editMode) {
                g_settingValues[g_selectedIndex] = g_editValue;
            }
            g_editMode = false;
        }
        lastSwitch2State = switch2State;
        lcd.clear();
        memset(g_lastLines, 0, sizeof(g_lastLines));
    }

    float knob = g_controlKnob2; // 0.0 - 1.0
    if (g_editMode) {
        int minVal = g_settingMin[g_selectedIndex];
        int maxVal = g_settingMax[g_selectedIndex];
        int range = maxVal - minVal;
        int newVal = minVal + (int)(knob * range + 0.5f);
        if (newVal < minVal) newVal = minVal;
        if (newVal > maxVal) newVal = maxVal;
        g_editValue = newVal;

        char line[21];
        const char *unit = g_settingUnits[g_selectedIndex];

        snprintf(line, sizeof(line), "%s (%s)",
                 g_settingFullNames[g_selectedIndex], unit);
        printLinePadded(0, line);

        snprintf(line, sizeof(line), "Range %d-%d %s", minVal, maxVal, unit);
        printLinePadded(1, line);

        snprintf(line, sizeof(line), "Prev:%4d %s", g_prevValue, unit);
        printLinePadded(2, line);

        snprintf(line, sizeof(line), "New: %4d %s",
                 g_editValue, unit);
        printLinePadded(3, line);

    } else {
        int newIndex = (int)(knob * kNumSettings);
        if (newIndex >= kNumSettings) newIndex = kNumSettings - 1;
        g_selectedIndex = newIndex;
        if (g_selectedIndex < g_topIndex) {
            g_topIndex = g_selectedIndex;
        } else if (g_selectedIndex > g_topIndex + 3) {
            g_topIndex = g_selectedIndex - 3;
        }

        printMainScreen();
    }
}
