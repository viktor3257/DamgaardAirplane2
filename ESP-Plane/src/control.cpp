#include <Arduino.h>
#include <math.h>

#include "control.h"
#include "communication.h"
#include "sensors.h"
#include "navigation.h"
#include "actuator_output.h"

// ===========================================================================
// PID Controller
// Time-aware integral leak, derivative low-pass, anti-windup with saturation.

struct PIDController {
    float kp = 0.f;
    float ki = 0.f;
    float kd = 0.f;

    float integral   = 0.f;
    float prevError  = 0.f;
    float dFiltered  = 0.f;
    unsigned long lastUpdate = 0;
};

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float wrap180f(float deg) {
    while (deg >  180.f) deg -= 360.f;
    while (deg < -180.f) deg += 360.f;
    return deg;
}

// Tunables for all PIDs (seconds)
static constexpr float kDtMin        = 0.001f;  // 1 ms
static constexpr float kDtMax        = 0.100f;  // 100 ms
static constexpr float kILeakTau     = 2.0f;    // integral "leak" time-constant
static constexpr float kDFilterTau   = 0.05f;   // derivative LPF time-constant

static void pidInit(PIDController &pid, float kp, float ki, float kd) {
    pid.kp = kp; pid.ki = ki; pid.kd = kd;
    pid.integral = 0.f;
    pid.prevError = 0.f;
    pid.dFiltered = 0.f;
    pid.lastUpdate = millis();
}

static float pidUpdateLimited(PIDController &pid, float error, float umin, float umax) {
    unsigned long now = millis();
    float dt = (now - pid.lastUpdate) / 1000.0f;
    pid.lastUpdate = now;
    dt = clampf(dt, kDtMin, kDtMax);

    // Integral with exponential leak, independent of loop rate
    const float leak = expf(-dt / kILeakTau);
    float i_next = pid.integral * leak + error * dt;

    // Derivative with simple first-order low-pass
    float derr = (error - pid.prevError) / dt;
    pid.prevError = error;
    pid.dFiltered += (derr - pid.dFiltered) * (dt / (dt + kDFilterTau));

    float u_unsat = pid.kp * error + pid.ki * i_next + pid.kd * pid.dFiltered;
    float u = clampf(u_unsat, umin, umax);

    // Anti-windup: only integrate if not pushing deeper into saturation
    bool pushing_up   = (u >= umax && u_unsat > u);
    bool pushing_down = (u <= umin && u_unsat < u);
    if (!(pushing_up || pushing_down)) {
        pid.integral = i_next;
    }
    return u;
}

// ===========================================================================
// Servo limiting and helpers

// Servos are centered at 90 deg and may deflect ±35 deg.
static constexpr float kMaxServoDeflectionDeg = 35.0f;

static inline float clampServoAngle(float angle) {
    float delta = clampf(angle - 90.f, -kMaxServoDeflectionDeg, kMaxServoDeflectionDeg);
    return 90.f + delta;
}

// Manual stick shaping (x in [0,1], returns [-1,1])
static float applyExpoDeadband(float x01, float expo, float deadband) {
    float x = clampf((x01 - 0.5f) * 2.f, -1.f, 1.f);
    // deadband
    if (fabsf(x) < deadband) x = 0.f;
    else x = (x > 0.f) ? (x - deadband) / (1.f - deadband)
                       : (x + deadband) / (1.f - deadband);
    // expo
    return x * (1.f - expo) + x * x * x * expo;
}

// ===========================================================================
// PID Instances (outer and inner loops)

static PIDController g_headingRollPID; // heading -> desired roll (deg)
static PIDController g_speedPitchPID;  // speed   -> desired pitch (deg)
static PIDController g_rollPID;        // roll error  -> aileron defl (deg)
static PIDController g_pitchPID;       // pitch error -> V-tail defl (deg)
static PIDController g_altitudePID;    // altitude -> throttle [0..1]

// Base gains (good starting point)
static constexpr float kHeadingRollKp = 1.0f;
static constexpr float kHeadingRollKi = 0.02f;
static constexpr float kHeadingRollKd = 0.20f;

static constexpr float kSpeedPitchKp = 0.9f;
static constexpr float kSpeedPitchKi = 0.02f;
static constexpr float kSpeedPitchKd = 0.10f;

static constexpr float kRollKp = 1.3f;
static constexpr float kRollKi = 0.00f;
static constexpr float kRollKd = 0.25f;

static constexpr float kPitchKp = 0.9f;
static constexpr float kPitchKi = 0.00f;
static constexpr float kPitchKd = 0.30f;

static constexpr float kAltitudeKp = 0.05f;
static constexpr float kAltitudeKi = 0.01f;
static constexpr float kAltitudeKd = 0.00f;

// ===========================================================================
// Responsiveness scaling: controller setting 0-100 -> scale factor 0.5-1.5
// Keep integral output continuous by scaling stored integral with Ki ratio.

static void scalePid(PIDController &pid, float baseKp, float baseKi, float baseKd, float scale) {
    float oldKi = pid.ki;
    pid.kp = baseKp * scale;
    pid.ki = baseKi * scale;
    pid.kd = baseKd * scale;
    if (oldKi > 0.f && pid.ki > 0.f) {
        pid.integral *= (oldKi / pid.ki);
    }
}

static void updatePidGainsForResponsiveness() {
    static int prevResponsiveness = -1;
    if (prevResponsiveness == g_settingResponsiveness) return;
    prevResponsiveness = g_settingResponsiveness;

    int resp = constrain(g_settingResponsiveness, 0, 100);
    float scale = 0.5f + resp / 100.0f; // 0.5 .. 1.5

    scalePid(g_headingRollPID, kHeadingRollKp, kHeadingRollKi, kHeadingRollKd, scale);
    scalePid(g_speedPitchPID,  kSpeedPitchKp,  kSpeedPitchKi,  kSpeedPitchKd,  scale);
    scalePid(g_rollPID,        kRollKp,        kRollKi,        kRollKd,        scale);
    scalePid(g_pitchPID,       kPitchKp,       kPitchKi,       kPitchKd,       scale);
    scalePid(g_altitudePID,    kAltitudeKp,    kAltitudeKi,    kAltitudeKd,    scale);
}

// ===========================================================================
// Misc constants

static constexpr float kAutonomousMaxBankDeg   = 45.0f;
static constexpr float kAutonomousMaxPitchDeg  = 20.0f;
static constexpr float kYawRollMixManual       = 0.35f;
static constexpr float kYawRollMixAuto         = 0.20f;

// Throttle value actually sent to the ESC
float g_outputThrottle = 0.0f;
// Desired angles the controller is currently aiming for (degrees)
float g_desiredRoll  = 0.0f;
float g_desiredPitch = 0.0f;

// Target airspeed used during radio-loss failsafe (m/s).
// Reset whenever the radio link is restored.
static float s_failsafeTargetSpeed = 0.0f;

// ===========================================================================
// Helpers

static void computeAutopilotSetpoints(float desiredHeading,
                                      float desiredAltitude,
                                      float desiredSpeed,
                                      float currentHeading,
                                      float currentAltitude,
                                      float currentSpeed,
                                      float &desiredRoll,
                                      float &desiredPitch,
                                      float &altitudeError) {
    float headingError = wrap180f(desiredHeading - currentHeading);
    desiredRoll = pidUpdateLimited(g_headingRollPID,
                                   headingError,
                                   -kAutonomousMaxBankDeg,
                                   +kAutonomousMaxBankDeg);

    float speedError = currentSpeed - desiredSpeed;
    desiredPitch = pidUpdateLimited(g_speedPitchPID,
                                    speedError,
                                    -kAutonomousMaxPitchDeg,
                                    +kAutonomousMaxPitchDeg);

    altitudeError = desiredAltitude - currentAltitude;
}

static void computeSurfaceAngles(float desiredRoll,
                                 float desiredPitch,
                                 float yawRollMixFactor,
                                 float &leftAileron,
                                 float &rightAileron,
                                 float &leftVTail,
                                 float &rightVTail) {
    // Inner-loop errors (deg)
    float rollError  = desiredRoll  - g_airplaneRoll;
    float pitchError = desiredPitch - g_airplanePitch;

    // Saturate inner-loop outputs to surface capability to avoid windup
    float rollCmdDeg  = pidUpdateLimited(g_rollPID,  rollError,
                                         -kMaxServoDeflectionDeg, +kMaxServoDeflectionDeg);
    float pitchCmdDeg = pidUpdateLimited(g_pitchPID, pitchError,
                                         -kMaxServoDeflectionDeg, +kMaxServoDeflectionDeg);

    float yawCmdDeg = rollCmdDeg * yawRollMixFactor;

    // Mix to surfaces, then clamp to servo angles around 90°
    leftAileron  = clampServoAngle(90.f + rollCmdDeg);
    rightAileron = clampServoAngle(90.f - rollCmdDeg);
    leftVTail    = clampServoAngle(90.f + pitchCmdDeg + yawCmdDeg);
    rightVTail   = clampServoAngle(90.f + pitchCmdDeg - yawCmdDeg);
}

static float computeThrottle(float altitudeError, float throttleMax) {
    // Altitude PID directly outputs throttle [0..throttleMax]
    return pidUpdateLimited(g_altitudePID, altitudeError, 0.f, throttleMax);
}

// ===========================================================================
// Public API

void initControl() {
    pidInit(g_headingRollPID, kHeadingRollKp, kHeadingRollKi, kHeadingRollKd);
    pidInit(g_speedPitchPID,  kSpeedPitchKp,  kSpeedPitchKi,  kSpeedPitchKd);
    pidInit(g_rollPID,        kRollKp,        kRollKi,        kRollKd);
    pidInit(g_pitchPID,       kPitchKp,       kPitchKi,       kPitchKd);
    pidInit(g_altitudePID,    kAltitudeKp,    kAltitudeKi,    kAltitudeKd);
    updatePidGainsForResponsiveness();
}

// ===========================================================================
// Mode handlers

static void writeNeutral() {
    g_outputThrottle = 0.f;
    writeActuatorOutputs(90.f, 90.f, 90.f, 90.f, g_outputThrottle);
}

static void updateManualControl() {
    // Stick shaping: expo and small deadband
    const float expo = 0.30f;
    const float dead = 0.04f;

    float rollStick  = applyExpoDeadband(g_controlJoyRX, expo, dead); // [-1,1]
    float pitchStick = applyExpoDeadband(g_controlJoyRY, expo, dead); // [-1,1]

    float desiredRoll  = rollStick  * 45.0f; // [-45,45] deg
    float desiredPitch = pitchStick * 20.0f; // [-20,20] deg

    g_desiredRoll  = desiredRoll;
    g_desiredPitch = desiredPitch;

    float leftAileron, rightAileron, leftVTail, rightVTail;
    computeSurfaceAngles(desiredRoll,
                         desiredPitch,
                         kYawRollMixManual,
                         leftAileron,
                         rightAileron,
                         leftVTail,
                         rightVTail);

    g_outputThrottle = clampf(g_controlKnob1, 0.f, 1.f); // manual throttle
    writeActuatorOutputs(leftAileron, rightAileron, leftVTail, rightVTail, g_outputThrottle);
}

// Failsafe behaviour: glide with throttle=0 and a gentle banked turn while
// holding a constant airspeed using pitch.  This runs whenever the radio link
// is lost and the plane is attempting to reconnect to the controller.
static void updateFailsafeControl() {
    const float kFailsafeBankDeg = 25.0f;  // gentle constant bank

    if (s_failsafeTargetSpeed <= 0.0f) {
        // Capture target speed on entry. Use configured circle speed if
        // available, otherwise use current airspeed.
        if (g_settingAutoCircleSpeed > 0) {
            s_failsafeTargetSpeed = (float)g_settingAutoCircleSpeed / 3.6f;
        } else {
            s_failsafeTargetSpeed = g_airplaneAirSpeed;
        }
    }

    float speedError = g_airplaneAirSpeed - s_failsafeTargetSpeed;
    float desiredPitch = pidUpdateLimited(g_speedPitchPID,
                                          speedError,
                                          -kAutonomousMaxPitchDeg,
                                          +kAutonomousMaxPitchDeg);

    float desiredRoll = kFailsafeBankDeg;

    g_desiredRoll  = desiredRoll;
    g_desiredPitch = desiredPitch;

    float leftAileron, rightAileron, leftVTail, rightVTail;
    computeSurfaceAngles(desiredRoll,
                         desiredPitch,
                         kYawRollMixAuto,
                         leftAileron,
                         rightAileron,
                         leftVTail,
                         rightVTail);

    g_outputThrottle = 0.0f;  // keep throttle at zero while gliding
    writeActuatorOutputs(leftAileron, rightAileron, leftVTail, rightVTail,
                         g_outputThrottle);
}

static void updateAutonomousControl() {
    float desiredRoll = 0.f;
    float desiredPitch = 0.f;
    float altitudeError = 0.f;

    computeAutopilotSetpoints(g_desiredHeading,
                              g_desiredAltitude,
                              g_desiredSpeed,
                              g_airplaneHeading,
                              g_airplaneHeight,
                              g_airplaneAirSpeed,
                              desiredRoll,
                              desiredPitch,
                              altitudeError);

    g_desiredRoll  = desiredRoll;
    g_desiredPitch = desiredPitch;

    float leftAileron, rightAileron, leftVTail, rightVTail;
    computeSurfaceAngles(desiredRoll,
                         desiredPitch,
                         kYawRollMixAuto,
                         leftAileron,
                         rightAileron,
                         leftVTail,
                         rightVTail);

    // Cap autonomous throttle at 0.8 and keep the altitude integrator aware of this limit
    g_outputThrottle = computeThrottle(altitudeError, 0.8f);
    writeActuatorOutputs(leftAileron, rightAileron, leftVTail, rightVTail, g_outputThrottle);
}

void updateControl() {
    updatePidGainsForResponsiveness();

    if (!g_radioLinkActive) {
        // Radio link lost: enter failsafe glide/turn while attempting to
        // reconnect.  Reset the captured speed once the link is restored.
        updateFailsafeControl();
        return;
    }

    s_failsafeTargetSpeed = 0.0f;

    switch (g_airplaneMode) {
        case AIRPLANE_MODE_MANUAL:
            updateManualControl();
            break;
        case AIRPLANE_MODE_CIRCLE:
            updateAutonomousControl();
            break;
        default:
            writeNeutral();
            break;
    }
}
