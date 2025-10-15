#include <Arduino.h>
#include "actuator_output.h"

static SimpleServo g_leftAileronServo;
static SimpleServo g_rightAileronServo;
static SimpleServo g_leftVTailServo;
static SimpleServo g_rightVTailServo;
static SimpleServo g_escServo;
static SimpleServo g_extraServo;

// Previous output commands used for simple smoothing filter
static float g_prevLeftAileron = 90.f;
static float g_prevRightAileron = 90.f;
static float g_prevLeftVTail = 90.f;
static float g_prevRightVTail = 90.f;
static float g_prevThrottle = 0.f;

// When set to false the ESC will never be attached or updated. This allows safe
// bench testing without the motor spinning.
static constexpr bool kEnableEscOutput = true;

static const int kLeftAileronPin = 27;  // U7
static const int kRightAileronPin = 32; // U8
static const int kLeftVTailPin = 25;    // U5
static const int kRightVTailPin = 26;   // U6
static const int kEscPin = 33;          // ESC
static const int kExtraServoPin = 14;   // U9

constexpr int OFFSET_LW = 90;   // left wing aileron   
constexpr int OFFSET_RW = 87;  // right wing aileron  
constexpr int OFFSET_LT = 100;   // left V-tail     -> Adjust value down = more up deflection
constexpr int OFFSET_RT = 90;   // right V-tail      

// Servo orientation adjustments. Set to -1 to reverse a servo.
static constexpr int kLeftAileronSign   = -1;  // Left aileron servo is flipped
static constexpr int kLeftVTailSign     = -1;  // Left tail servo is flipped
static constexpr int kRightAileronSign  = 1;
static constexpr int kRightVTailSign    = 1;

static constexpr int SERVO_MIN_US = 500;
static constexpr int SERVO_MAX_US = 2500;

static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_MAX_US = 2000;
static constexpr float kEscOutputScale = 0.6f; // Limit commanded throttle to 60% of ESC range

static constexpr int kServoTravelLimit = 30; // degrees from neutral

static int applyOffset(float angle, int offset) {
    int value = static_cast<int>(angle + 0.5f) + offset - 90;

    // Limit servo travel to within +/-kServoTravelLimit degrees of the neutral (offset) angle
    int minValue = offset - kServoTravelLimit;
    int maxValue = offset + kServoTravelLimit;
    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;

    if (value < 0) value = 0;
    if (value > 180) value = 180;
    return value;
}

static int angleToMicroseconds(int angle)
{
    int us = SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return us;
}

static float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float applySign(float angle, int sign) {
    float cmd = angle - 90.f;
    cmd *= static_cast<float>(sign);
    return 90.f + cmd;
}

static void writeServoWithOffset(SimpleServo &servo, float angle, int offset) {
    int adjAngle = applyOffset(angle, offset);
    servo.writeMicroseconds(angleToMicroseconds(adjAngle));
}

static int throttleToMicroseconds(float throttle) {
    float scaledThrottle = clampFloat(throttle, 0.f, 1.f) * kEscOutputScale;
    int us = static_cast<int>(ESC_MIN_US + scaledThrottle * (ESC_MAX_US - ESC_MIN_US) + 0.5f);
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    const int maxEscUs = static_cast<int>(ESC_MIN_US + (ESC_MAX_US - ESC_MIN_US) * kEscOutputScale + 0.5f);
    if (us > maxEscUs) {
        us = maxEscUs;
    }
    return us;
}

void initActuatorOutput() {
    g_leftAileronServo.attach(kLeftAileronPin, 0, SERVO_MIN_US, SERVO_MAX_US);
    g_rightAileronServo.attach(kRightAileronPin, 1, SERVO_MIN_US, SERVO_MAX_US);
    g_leftVTailServo.attach(kLeftVTailPin, 2, SERVO_MIN_US, SERVO_MAX_US);
    g_rightVTailServo.attach(kRightVTailPin, 3, SERVO_MIN_US, SERVO_MAX_US);
    if (kEnableEscOutput) {
        g_escServo.attach(kEscPin, 4, ESC_MIN_US, ESC_MAX_US);
    }
    g_extraServo.attach(kExtraServoPin, 5, SERVO_MIN_US, SERVO_MAX_US);

    g_leftAileronServo.writeMicroseconds(angleToMicroseconds(OFFSET_LW));
    g_rightAileronServo.writeMicroseconds(angleToMicroseconds(OFFSET_RW));
    g_leftVTailServo.writeMicroseconds(angleToMicroseconds(OFFSET_LT));
    g_rightVTailServo.writeMicroseconds(angleToMicroseconds(OFFSET_RT));
    if (kEnableEscOutput) {
        g_escServo.writeMicroseconds(throttleToMicroseconds(0.f));
    }
    g_extraServo.writeMicroseconds(angleToMicroseconds(90));

    // Initialize smoothing state
    g_prevLeftAileron = 90.f;
    g_prevRightAileron = 90.f;
    g_prevLeftVTail = 90.f;
    g_prevRightVTail = 90.f;
    g_prevThrottle = 0.f;
}

void writeActuatorOutputs(float leftAileron,
                          float rightAileron,
                          float leftVTail,
                          float rightVTail,
                          float throttle) {
    const float kSmoothFactor = 0.25f;
    auto smooth = [](float &prev, float target, float alpha) {
        prev += alpha * (target - prev);
        return prev;
    };

    float smLeftAileron  = smooth(g_prevLeftAileron, leftAileron, kSmoothFactor);
    float smRightAileron = smooth(g_prevRightAileron, rightAileron, kSmoothFactor);
    float smLeftVTail    = smooth(g_prevLeftVTail, leftVTail, kSmoothFactor);
    float smRightVTail   = smooth(g_prevRightVTail, rightVTail, kSmoothFactor);
    float smThrottle     = smooth(g_prevThrottle, throttle, kSmoothFactor);

    writeServoWithOffset(g_leftAileronServo,
                         applySign(smLeftAileron, kLeftAileronSign),
                         OFFSET_LW);
    writeServoWithOffset(g_rightAileronServo,
                         applySign(smRightAileron, kRightAileronSign),
                         OFFSET_RW);
    writeServoWithOffset(g_leftVTailServo,
                         applySign(smLeftVTail, kLeftVTailSign),
                         OFFSET_LT);
    writeServoWithOffset(g_rightVTailServo,
                         applySign(smRightVTail, kRightVTailSign),
                         OFFSET_RT);
    if (kEnableEscOutput) {
        g_escServo.writeMicroseconds(throttleToMicroseconds(smThrottle));
    }
    (void)g_extraServo;
}
