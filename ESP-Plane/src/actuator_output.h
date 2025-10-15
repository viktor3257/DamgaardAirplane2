#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  SimpleServo -- minimal wrapper around the ESP32 LEDC API
// ---------------------------------------------------------------------------
class SimpleServo {
public:
    SimpleServo()
        : m_pin(-1), m_channel(-1), m_minUs(500), m_maxUs(2500), m_attached(false) {}

    void attach(int pin, int channel, int minUs = 500, int maxUs = 2500, int freq = 50) {
        m_pin = pin;
        m_channel = channel;
        m_minUs = minUs;
        m_maxUs = maxUs;
        ledcSetup(m_channel, freq, 16);
        ledcAttachPin(m_pin, m_channel);
        m_attached = true;
    }

    void writeMicroseconds(int us) {
        if (!m_attached) return;
        us = constrain(us, m_minUs, m_maxUs);
        uint32_t duty = (uint32_t)((us * 65535UL) / 20000UL);
        ledcWrite(m_channel, duty);
    }

private:
    int m_pin;
    int m_channel;
    int m_minUs;
    int m_maxUs;
    bool m_attached;
};

void initActuatorOutput();
void writeActuatorOutputs(float leftAileron,
                          float rightAileron,
                          float leftVTail,
                          float rightVTail,
                          float throttle);
