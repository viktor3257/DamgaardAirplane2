#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <string.h>
#include <stdio.h>
#include "sensors.h"
#include "communication.h"

float g_airplaneHeight = 0.0f;
float g_airplaneHeading = 0.0f;
float g_airplaneAirSpeed = 0.0f;
float g_airplaneGroundSpeed = 0.0f;
float g_airplaneClimbRate = 0.0f;
float g_airplanePitch = 0.0f;
float g_airplaneRoll = 0.0f;
float g_airplaneBatteryPercent = 100.0f;
float g_airplaneCurrentA = 0.0f;
float g_airplaneCurrentSensorV = 0.0f;

double g_airplaneLatitude = 0.0;
double g_airplaneLongitude = 0.0;
double g_airplaneHomeLatitude = 0.0;
double g_airplaneHomeLongitude = 0.0;
double g_circleTargetLatitude = 55.847649;
double g_circleTargetLongitude = 9.404065;

double g_areaSearchRouteLat[kMaxAreaSearchPoints] = {0};
double g_areaSearchRouteLon[kMaxAreaSearchPoints] = {0};
uint16_t g_areaSearchRouteCount = 0;
float g_areaSearchRouteAge = 0.0f;

bool g_airplaneReceivesDataFromRaspberryPi = false;

static Adafruit_BMP280 g_bmp;
static Adafruit_BNO055 g_bno = Adafruit_BNO055(55, 0x29);

// Flags indicating if optional sensors were successfully initialized.
static bool g_bmpAvailable = false;
static bool g_bnoAvailable = false;

// Offsets to ensure pitch and roll read zero when the plane is level
static constexpr float kPitchOffsetDegrees = 11.19f;
static constexpr float kRollOffsetDegrees = 1.31f;

// MS4525DO airspeed sensor address
static constexpr uint8_t kAirspeedAddress = 0x28;

// Airspeed offset calculated during startup
static float g_airspeedOffset = 0.0f;
static unsigned long g_airspeedOffsetStart = 0;
static float g_airspeedOffsetSum = 0.0f;
static uint32_t g_airspeedOffsetSamples = 0;
static bool g_airspeedOffsetReady = false;

// Current sensor configuration
static constexpr int   kCurrentPin = 39;          // ACS758 on ADC pin 39
static const float     kBatteryCapacityMah = 6000.0f; // 4S 6000 mAh pack
static const float     kCurrentSensorZero = 1.453f;   // Observed voltage at 0 A
static const float     kCurrentSensorSensitivity = 0.0264f; // Volts per Amp at 3.3V
static const float     kAdcVoltage = 3.3f;        // ADC reference voltage
static const int       kAdcResolution = 4095;     // 12-bit ADC
static unsigned long   g_lastCurrentTime = 0;
static float           g_initialBatteryPercent = 100.f;
static float           g_consumedMah = 0.0f;

static float g_pressureReference = 0.f;
static unsigned long g_lastPiGpsTime = 0;
static const unsigned long kPiGpsTimeout = 2000; // ms

static char g_piBuf[128];
static size_t g_piIdx = 0;
static bool g_routeReceiving = false;
static unsigned long g_routeReceivedTime = 0;

static void parsePiLine(const char* line) {
    if (strcmp(line, "ROUTE_START") == 0) {
        g_routeReceiving = true;
        g_areaSearchRouteCount = 0;
        return;
    }
    if (strcmp(line, "ROUTE_END") == 0) {
        g_routeReceiving = false;
        g_routeReceivedTime = millis();
        g_areaSearchRouteAge = 0.f;
        return;
    }

    if (g_routeReceiving) {
        if (g_areaSearchRouteCount < kMaxAreaSearchPoints) {
            double lat = 0.0, lon = 0.0;
            if (sscanf(line, "%lf,%lf", &lat, &lon) == 2) {
                size_t idx = g_areaSearchRouteCount++;
                g_areaSearchRouteLat[idx] = lat;
                g_areaSearchRouteLon[idx] = lon;
            }
        }
        return;
    }

    double lat = 0.0, lon = 0.0;
    double tlat = 0.0, tlon = 0.0;
    double heading = 0.0;

    int count = sscanf(line, "%lf,%lf,%lf,%lf,%lf", &lat, &lon, &tlat, &tlon, &heading);
    if (count >= 2) {
        g_airplaneLatitude = lat;
        g_airplaneLongitude = lon;
        if (count >= 4) {
            g_circleTargetLatitude = tlat;
            g_circleTargetLongitude = tlon;
        }
        if (count >= 5) {
            g_airplaneHeading = heading;
        }
    } else {
        const char* pLat = strstr(line, "\"lat\"");
        const char* pLng = strstr(line, "\"lng\"");
        if (!pLng) pLng = strstr(line, "\"lon\"");
        const char* pTLat = strstr(line, "\"tlat\"");
        if (!pTLat) pTLat = strstr(line, "\"target_lat\"");
        const char* pTLng = strstr(line, "\"tlng\"");
        if (!pTLng) pTLng = strstr(line, "\"target_lng\"");
        if (!pTLng) pTLng = strstr(line, "\"target_lon\"");
        const char* pHeading = strstr(line, "\"heading\"");

        if (pLat) pLat = strchr(pLat, ':');
        if (pLng) pLng = strchr(pLng, ':');
        if (pTLat) pTLat = strchr(pTLat, ':');
        if (pTLng) pTLng = strchr(pTLng, ':');
        if (pHeading) pHeading = strchr(pHeading, ':');

        if (pLat && pLng) {
            lat = atof(pLat + 1);
            lon = atof(pLng + 1);
            g_airplaneLatitude = lat;
            g_airplaneLongitude = lon;
        }
        if (pTLat && pTLng) {
            tlat = atof(pTLat + 1);
            tlon = atof(pTLng + 1);
            g_circleTargetLatitude = tlat;
            g_circleTargetLongitude = tlon;
        }
        if (pHeading) {
            heading = atof(pHeading + 1);
            g_airplaneHeading = heading;
        }
    }

    g_airplaneReceivesDataFromRaspberryPi = true;
    g_lastPiGpsTime = millis();
}

static void readPiGps() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            if (g_piIdx > 0) {
                g_piBuf[g_piIdx] = '\0';
                parsePiLine(g_piBuf);
                g_piIdx = 0;
            }
        } else if (g_piIdx < sizeof(g_piBuf) - 1) {
            g_piBuf[g_piIdx++] = c;
        }
    }

    if (g_airplaneReceivesDataFromRaspberryPi && g_lastPiGpsTime != 0) {
        unsigned long now = millis();
        if (now - g_lastPiGpsTime > kPiGpsTimeout) {
            g_airplaneReceivesDataFromRaspberryPi = false;
            g_lastPiGpsTime = 0;
        }
    }

    if (g_routeReceivedTime != 0) {
        unsigned long now = millis();
        g_areaSearchRouteAge = (now - g_routeReceivedTime) / 1000.0f;
    }
}


static void readBNO055() {
    if (!g_bnoAvailable) {
        return;
    }

    auto vec = g_bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // BNO055 reports pitch as negative when the nose is raised.  Invert the sign
    // so that a positive value corresponds to pitching up, matching the rest of
    // the control code's expectations.
    g_airplanePitch = -vec.y() + 10.f - kPitchOffsetDegrees;  // Positive when nose up
    g_airplaneRoll = vec.z() + kRollOffsetDegrees;            // Positive when rolling right
}

static void readBMP280() {
    if (!g_bmpAvailable) {
        return;
    }

    float rawPressure = g_bmp.readPressure();
    float pressure = rawPressure - g_pressureReference;
    const float airDensity = 1.225f; // kg/m^3 at sea level
    const float gravity = 9.80665f;  // m/s^2
    g_airplaneHeight = -pressure / (airDensity * gravity);
}

static void readAirspeed() {
    Wire.requestFrom(kAirspeedAddress, (uint8_t)4);
    if (Wire.available() < 4) {
        return;
    }

    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    (void)b2;
    (void)b3;

    uint8_t status = b0 >> 6;
    if (status == 2 || status == 3) {
        return;
    }

    int16_t dp_raw = ((b0 & 0x3F) << 8) | b1;
    const float P_min = -1.0f; // psi
    const float P_max = 1.0f;  // psi
    float diff_press_psi = -(((float)dp_raw - 0.1f * 16383.0f) *
                             (P_max - P_min) / (0.8f * 16383.0f) + P_min);
    float press_pa = diff_press_psi * 6894.757f; // Pa
    if (press_pa < 0.0f) {
        press_pa = -press_pa;
    }

    const float airDensity = 1.225f; // kg/m^3 at sea level
    float newAirSpeed = sqrtf(2.0f * press_pa / airDensity);

    unsigned long now = millis();
    if (!g_airspeedOffsetReady) {
        if (g_airspeedOffsetStart == 0) {
            g_airspeedOffsetStart = now;
        }
        g_airspeedOffsetSum += newAirSpeed;
        g_airspeedOffsetSamples++;
        if (now - g_airspeedOffsetStart >= 3000 && g_airspeedOffsetSamples > 0) {
            g_airspeedOffset = g_airspeedOffsetSum / g_airspeedOffsetSamples;
            g_airspeedOffsetReady = true;
        }
        g_airplaneAirSpeed = 0.0f;
        return;
    }

    newAirSpeed -= g_airspeedOffset;
    if (newAirSpeed < 0.0f) {
        newAirSpeed = 0.0f;
    }

    // Simple exponential moving average to smooth jitter
    static bool initialized = false;
    constexpr float kAlpha = 0.1f; // smoothing factor [0..1]
    if (!initialized) {
        g_airplaneAirSpeed = newAirSpeed;
        initialized = true;
    } else {
        g_airplaneAirSpeed = g_airplaneAirSpeed * (1.0f - kAlpha) +
                             newAirSpeed * kAlpha;
    }
}

static void readBattery() {
    // Update initial battery level if we received new settings
    if (g_settingInitialBatteryPercent > 0 &&
        g_initialBatteryPercent != g_settingInitialBatteryPercent) {
        g_initialBatteryPercent = g_settingInitialBatteryPercent;
        g_consumedMah = 0.0f;
    }

    unsigned long now = millis();
    if (g_lastCurrentTime == 0) {
        g_lastCurrentTime = now;
    }
    unsigned long dtMs = now - g_lastCurrentTime;
    g_lastCurrentTime = now;

    int raw = analogRead(kCurrentPin);
    float voltage = raw * kAdcVoltage / kAdcResolution;
    g_airplaneCurrentSensorV = voltage;
    // The ACS758 outputs a lower voltage as current increases. Flip the sign so
    // positive current corresponds to a voltage drop across the sensor.
    g_airplaneCurrentA = (kCurrentSensorZero - voltage) / kCurrentSensorSensitivity;

    float hours = dtMs / 3600000.0f;
    g_consumedMah += g_airplaneCurrentA * hours * 1000.0f;

    float startMah = kBatteryCapacityMah * (g_initialBatteryPercent / 100.0f);
    float remainingMah = startMah - g_consumedMah;
    if (remainingMah < 0.0f) remainingMah = 0.0f;
    g_airplaneBatteryPercent = (remainingMah / kBatteryCapacityMah) * 100.0f;
}

void initSensors() {
    Wire.begin();

    g_bnoAvailable = g_bno.begin();
    if (g_bnoAvailable) {
        g_bno.setExtCrystalUse(true);
    }

    g_bmpAvailable = g_bmp.begin();
    if (!g_bmpAvailable) {
        g_pressureReference = 0.f;
    } else {
        g_pressureReference = g_bmp.readPressure();
    }

    pinMode(kCurrentPin, INPUT);
    g_initialBatteryPercent = (g_settingInitialBatteryPercent > 0)
                                  ? g_settingInitialBatteryPercent
                                  : g_airplaneBatteryPercent;
    g_consumedMah = 0.0f;
    g_lastCurrentTime = millis();
}

void readSensors() {
    readPiGps();
    readBNO055();
    readBMP280();
    readAirspeed();
    readBattery();
}
