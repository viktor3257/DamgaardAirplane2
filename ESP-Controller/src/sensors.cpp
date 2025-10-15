#include <Arduino.h>
#include "sensors.h"
#include "communication.h"

// Button pins
static const int kButton1Pin = 13;
static const int kButton2Pin = 14;
static const int kButton3Pin = 33;
static const int kButton4Pin = 32;

// Potentiometer pins
static const int kPot1Pin = 36;
static const int kPot2Pin = 39;

// Switch pins (active low)
static const int kSwitch1Pin = 16;
static const int kSwitch2Pin = 17;

// Joystick pins
static const int kJoy1XPin = 34;
static const int kJoy1YPin = 35;


float g_controlKnob1 = 0.0f;
float g_controlKnob2 = 0.0f;
float g_controlJoyRX = 0.0f;
float g_controlJoyRY = 0.0f;
uint8_t g_switch1 = 0;
uint8_t g_switch2 = 0;

// Raw ADC readings
static int g_pot1Raw = 0, g_pot2Raw = 0;
static int g_joy1XRaw = 0, g_joy1YRaw = 0;


// Button interrupt flags
static volatile bool g_button1Flag = false;
static volatile bool g_button2Flag = false;
static volatile bool g_button3Flag = false;
static volatile bool g_button4Flag = false;
static volatile unsigned long g_button1Last = 0;
static volatile unsigned long g_button2Last = 0;
static volatile unsigned long g_button3Last = 0;
static volatile unsigned long g_button4Last = 0;
static const unsigned long kDebounceMs = 200;

// ---------------------------------------------------------------------------
//  Joystick handling
// ---------------------------------------------------------------------------

// Piecewise linear mapping that allows asymmetric centre positions.
// The inputs are raw ADC values and a pre-measured mid point.
static int mapJoystick(int raw, int minVal, int midVal, int maxVal, bool invert)
{
    int out;
    if (raw <= midVal) {
        out = map(raw, minVal, midVal, 0, 50);
    } else {
        out = map(raw, midVal, maxVal, 50, 99);
    }
    out = constrain(out, 0, 99);
    return invert ? (99 - out) : out;
}

// Estimated joystick limits based on observed extremes
#define J1X_MIN  490
#define J1X_MAX  3150
#define J1Y_MIN  530
#define J1Y_MAX  3590

// Midpoints are calibrated at start-up
static int g_j1xMid = 2048;
static int g_j1yMid = 2048;

static int calibrateAxis(int pin, int samples = 20)
{
    long sum = 0;
    for (int i = 0; i < samples; ++i) {
        sum += analogRead(pin);
        delay(5);
    }
    return static_cast<int>(sum / samples);
}

static void calibrateJoysticks()
{
    g_j1xMid = calibrateAxis(kJoy1XPin);
    g_j1yMid = calibrateAxis(kJoy1YPin);
}

// Interrupt handlers with debounce
static void IRAM_ATTR onButton1() {
    unsigned long now = millis();
    if (now - g_button1Last > kDebounceMs) {
        g_button1Flag = true;
        g_button1Last = now;
    }
}
static void IRAM_ATTR onButton2() {
    unsigned long now = millis();
    if (now - g_button2Last > kDebounceMs) {
        g_button2Flag = true;
        g_button2Last = now;
    }
}
static void IRAM_ATTR onButton3() {
    unsigned long now = millis();
    if (now - g_button3Last > kDebounceMs) {
        g_button3Flag = true;
        g_button3Last = now;
    }
}
static void IRAM_ATTR onButton4() {
    unsigned long now = millis();
    if (now - g_button4Last > kDebounceMs) {
        g_button4Flag = true;
        g_button4Last = now;
    }
}

void initSensors()
{
    // Use internal pull-up resistors for the buttons. The buttons are wired to
    // connect the input pin to ground when pressed, so we trigger interrupts on
    // the falling edge.
    pinMode(kButton1Pin, INPUT_PULLUP);
    pinMode(kButton2Pin, INPUT_PULLUP);
    pinMode(kButton3Pin, INPUT_PULLUP);
    pinMode(kButton4Pin, INPUT_PULLUP);

    pinMode(kSwitch1Pin, INPUT_PULLUP);
    pinMode(kSwitch2Pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(kButton1Pin), onButton1, FALLING);
    attachInterrupt(digitalPinToInterrupt(kButton2Pin), onButton2, FALLING);
    attachInterrupt(digitalPinToInterrupt(kButton3Pin), onButton3, FALLING);
    attachInterrupt(digitalPinToInterrupt(kButton4Pin), onButton4, FALLING);

    calibrateJoysticks();

    // Ensure the airplane starts in neutral mode on boot
    g_airplaneMode        = AIRPLANE_MODE_NEUTRAL;
    g_desiredAirplaneMode = AIRPLANE_MODE_NEUTRAL;
}

void readSensors()
{
    g_pot1Raw  = analogRead(kPot1Pin);
    g_pot2Raw  = analogRead(kPot2Pin);
    g_joy1XRaw = analogRead(kJoy1XPin);
    g_joy1YRaw = analogRead(kJoy1YPin);

    int pot1Val = map(g_pot1Raw, 0, 4095, 0, 99);
    int pot2Val = map(g_pot2Raw, 0, 4095, 0, 99);

    int jxVal = mapJoystick(g_joy1XRaw, J1X_MIN, g_j1xMid, J1X_MAX, false);
    int jyVal = mapJoystick(g_joy1YRaw, J1Y_MIN, g_j1yMid, J1Y_MAX, false);

    g_controlKnob1 = pot1Val / 99.0f;
    g_controlKnob2 = pot2Val / 99.0f;
    g_controlJoyRX = jxVal / 99.0f;
    g_controlJoyRY = jyVal / 99.0f;
    g_switch1 = (digitalRead(kSwitch1Pin) == LOW) ? 1 : 0;
    g_switch2 = (digitalRead(kSwitch2Pin) == LOW) ? 1 : 0;

    // Update desired airplane mode based on button presses
    if (g_button1Flag) {
        g_button1Flag = false;
        g_desiredAirplaneMode = AIRPLANE_MODE_MANUAL;
    } else if (g_button2Flag) {
        g_button2Flag = false;
        g_desiredAirplaneMode = AIRPLANE_MODE_CIRCLE;
    } else if (g_button3Flag) {
        g_button3Flag = false;
        g_desiredAirplaneMode = AIRPLANE_MODE_AREA_SEARCH;
    }
    // Button 4 is handled by the display/menu system.
}

void updateServo() {}

bool button4Pressed() {
    static unsigned long lastHandled = 0;
    unsigned long now = millis();
    if (g_button4Flag && now - lastHandled > kDebounceMs) {
        g_button4Flag = false;
        lastHandled = now;
        return true;
    }
    // Clear any extra flags so they don't accumulate while in other modes
    g_button4Flag = false;
    return false;
}

