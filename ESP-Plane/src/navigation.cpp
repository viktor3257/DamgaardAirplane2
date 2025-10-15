#include <Arduino.h>
#include <math.h>
#include "navigation.h"
#include "communication.h"
#include "sensors.h"

float g_desiredHeading = 0.0f;
float g_desiredAltitude = 0.0f;
float g_desiredSpeed = 0.0f;

// Circling target position (center of the circle)
static const float  kCircleRadius    = 30.0f;      // metres

static constexpr float kEarthRadius = 6371000.0f;             // metres

static inline float deg2rad(float deg) { return deg * (float)M_PI / 180.0f; }
static inline float rad2deg(float rad) { return rad * 180.0f / (float)M_PI; }

// Utility conversions

static void navigateCircle() {
    // Clockwise intercept/orbit guidance using GPS coordinates
    const float gainDegPerM = -2.0f;
    const float maxOutDeg   = 45.0f;
    const float epsR        = 1e-6f;

    double planeLat  = g_airplaneLatitude;
    double planeLon  = g_airplaneLongitude;
    double centerLat = g_circleTargetLatitude;
    double centerLon = g_circleTargetLongitude;

    double dLat = (planeLat - centerLat) * DEG_TO_RAD;
    double dLon = (planeLon - centerLon) * DEG_TO_RAD;
    double avg  = (planeLat + centerLat) * 0.5 * DEG_TO_RAD;

    float dy = static_cast<float>(dLat * kEarthRadius);              // north
    float dx = static_cast<float>(dLon * kEarthRadius * cos(avg));   // east

    float d = hypotf(dx, dy);

    bool currentlyCircling = fabsf(d - kCircleRadius) <= 20.0f;

    float desiredRad;

    if (d < epsR) {
        desiredRad = M_PI;      // due west (CW tangent)
    } else if (d > kCircleRadius + epsR) {
        float theta = atan2f(dy, dx);
        float alpha = acosf(fmaxf(-1.0f, fminf(1.0f, kCircleRadius / d)));

        float phi1 = theta + alpha;
        float phi2 = theta - alpha;

        float T1x = kCircleRadius * cosf(phi1);
        float T1y = kCircleRadius * sinf(phi1);
        float T2x = kCircleRadius * cosf(phi2);
        float T2y = kCircleRadius * sinf(phi2);

        float vCx = -dx;
        float vCy = -dy;

        float v1x = T1x - dx;
        float v1y = T1y - dy;
        float v2x = T2x - dx;
        float v2y = T2y - dy;

        float cross1 = vCx * v1y - vCy * v1x;
        float cross2 = vCx * v2y - vCy * v2x;

        if (cross2 > cross1) {
            desiredRad = atan2f(v2y, v2x);
        } else {
            desiredRad = atan2f(v1y, v1x);
        }
    } else {
        float tangentRad = atan2f(-dx, dy);
        float insideErr  = fmaxf(0.0f, kCircleRadius - d);
        float extraDeg   = gainDegPerM * insideErr;
        extraDeg = fmaxf(-maxOutDeg, fminf(maxOutDeg, extraDeg));
        desiredRad = tangentRad - deg2rad(extraDeg);
    }

    float headingDeg = fmodf(90.0f - rad2deg(desiredRad), 360.0f);
    if (headingDeg < 0.0f) headingDeg += 360.0f;
    g_desiredHeading = headingDeg;

    g_desiredAltitude = static_cast<float>(g_settingAutoAltitude);

    float circleSpeedMps = static_cast<float>(g_settingAutoCircleSpeed) / 3.6f;
    float taxiSpeedMps   = static_cast<float>(g_settingAutoTaxiSpeed) / 3.6f;
    g_desiredSpeed = currentlyCircling ? circleSpeedMps : taxiSpeedMps;
}

static void navigateAreaSearch() {
    static uint16_t areaIdx = 0;
    static float lastRouteAge = -1.f;

    if (g_areaSearchRouteAge < lastRouteAge) {
        areaIdx = 0;  // new route loaded
    }
    lastRouteAge = g_areaSearchRouteAge;

    uint16_t count = g_areaSearchRouteCount;
    float desiredHeading = g_airplaneHeading;

    if (count != 0) {
        if (areaIdx >= count) {
            areaIdx = 0;
        }

        double tgtLat = g_areaSearchRouteLat[areaIdx];
        double tgtLon = g_areaSearchRouteLon[areaIdx];

        auto distanceMeters = [](double lat1, double lon1,
                                 double lat2, double lon2) {
            double dLat = deg2rad(lat2 - lat1);
            double dLon = deg2rad(lon2 - lon1);
            double lat1r = deg2rad(lat1);
            double lat2r = deg2rad(lat2);
            double a = sin(dLat * 0.5) * sin(dLat * 0.5) +
                       cos(lat1r) * cos(lat2r) *
                       sin(dLon * 0.5) * sin(dLon * 0.5);
            double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
            return static_cast<float>(kEarthRadius * c);
        };

        auto bearingDegrees = [](double lat1, double lon1,
                                 double lat2, double lon2) {
            double lat1r = deg2rad(lat1);
            double lat2r = deg2rad(lat2);
            double dLon = deg2rad(lon2 - lon1);
            double y = sin(dLon) * cos(lat2r);
            double x = cos(lat1r) * sin(lat2r) -
                       sin(lat1r) * cos(lat2r) * cos(dLon);
            double brng = atan2(y, x);
            float heading = rad2deg(brng);
            if (heading < 0.0f)
                heading += 360.0f;
            return heading;
        };

        float dist = distanceMeters(g_airplaneLatitude,
                                   g_airplaneLongitude,
                                   tgtLat, tgtLon);
        if (dist < 20.0f) {
            areaIdx++;
            if (areaIdx >= count)
                areaIdx = 0;
            tgtLat = g_areaSearchRouteLat[areaIdx];
            tgtLon = g_areaSearchRouteLon[areaIdx];
        }

        desiredHeading = bearingDegrees(g_airplaneLatitude,
                                        g_airplaneLongitude,
                                        tgtLat, tgtLon);
    }

    g_desiredHeading = desiredHeading;
    g_desiredAltitude = static_cast<float>(g_settingAutoAltitude);
    g_desiredSpeed    = static_cast<float>(g_settingAutoTaxiSpeed) / 3.6f;
}


void initNavigation() {
    // Setup navigation algorithms
}

void updateNavigation() {
    if (g_airplaneMode == AIRPLANE_MODE_CIRCLE) {
        navigateCircle();
    } else if (g_airplaneMode == AIRPLANE_MODE_AREA_SEARCH) {
        navigateAreaSearch();
    }
}
