#include "outputs.hpp"

#include "config.hpp"
#include "pins.hpp"

namespace {

LightMode lightMode = AUTO_LIGHT_MODE ? LightMode::AUTO
                                      : LightMode::FORCED_OFF;
uint32_t pulseEndMs = 0;

void writeHeadlight(bool on) {
    digitalWrite(Pins::HEADLIGHT, (on == (HEADLIGHT_ACTIVE_HIGH != 0)) ? HIGH : LOW);
}

void writeBuzzer(bool on) {
    digitalWrite(Pins::ALARM, (on == (BUZZER_ACTIVE_HIGH != 0)) ? HIGH : LOW);
}

bool timeBefore(uint32_t nowMs, uint32_t endMs) {
    return static_cast<int32_t>(nowMs - endMs) < 0;
}

}  // namespace

void outputsBegin() {
    pinMode(Pins::HEADLIGHT, OUTPUT);
    pinMode(Pins::ALARM, OUTPUT);
    writeHeadlight(false);
    writeBuzzer(false);
}

void outputsSetLightMode(LightMode mode) {
    lightMode = mode;
}

LightMode outputsGetLightMode() {
    return lightMode;
}

void outputsPulseAlarm(uint32_t nowMs, uint16_t durationMs) {
    const uint32_t requestedEnd = nowMs + durationMs;
    if (!timeBefore(requestedEnd, pulseEndMs)) {
        pulseEndMs = requestedEnd;
    }
}

void outputsUpdate(bool dark, bool warningActive, bool emergencyActive, uint32_t nowMs) {
    bool headlightsOn = false;
    if (lightMode == LightMode::FORCED_ON) {
        headlightsOn = true;
    } else if (lightMode == LightMode::AUTO) {
        headlightsOn = dark;
    }
    writeHeadlight(headlightsOn);

    bool buzzerOn = false;
    if (emergencyActive) {
        buzzerOn = ((nowMs / EMERGENCY_BEEP_HALF_PERIOD_MS) & 1U) == 0U;
    } else if (timeBefore(nowMs, pulseEndMs)) {
        buzzerOn = true;
    } else if (warningActive) {
        buzzerOn = (nowMs % WARNING_BEEP_PERIOD_MS) < WARNING_BEEP_ON_MS;
    }
    writeBuzzer(buzzerOn);
}
