#pragma once

#include <Arduino.h>

// MQ-135 state used by the safety layer and telemetry output. Warming is kept
// separate from normal so the operator can see that the ADC is not calibrated.
enum GasState : uint8_t {
    GAS_WARMING,
    GAS_NORMAL,
    GAS_WARNING,
    GAS_CRITICAL
};

// Latest values and validity flags shared with the main control loop. Integer
// deci-units keep decimal temperature and humidity output compact on the Uno.
struct SensorData {
    // Median-filtered ultrasonic distance and its validation result.
    uint16_t distanceCm;
    // Raw and low-pass-filtered MQ-135 ADC values.
    uint16_t gasRaw;
    uint16_t gasFiltered;
    // Sound-window amplitude, light ADC value, and DHT11 readings.
    uint16_t soundAmplitude;
    uint16_t lightRaw;
    int16_t temperatureDeciC;
    uint16_t humidityDeciPct;
    // Filtered accelerometer angles used by the tilt safety checks.
    int16_t pitchDeg;
    int16_t rollDeg;
    // Interpreted sensor states used in messages and output control.
    GasState gasState;
    bool distanceValid;
    bool dhtValid;
    bool temperatureCritical;
    bool mpuValid;
    bool tiltCritical;
    bool pirMotion;
    bool dark;
};

// Pending events are bit flags so one sensor pass can report sound and motion
// without allocating a message buffer on the Uno.
const uint8_t SENSOR_EVENT_SOUND = 1 << 0;
const uint8_t SENSOR_EVENT_MOTION = 1 << 1;

// Start and run the scheduled sensor service. The main loop supplies its own
// time and motion state so this module does not block on a scheduler.
void sensorsBegin();
void sensorsUpdate(uint32_t nowMs, bool motorsMoving, bool scanMode);
const SensorData& sensorsGetData();

// Sound-window and scan helpers used by the command and telemetry paths.
// A scan requests a synchronized snapshot without changing normal scheduling.
void sensorsStartSoundWindow(uint32_t nowMs);
bool sensorsSoundWindowActive();
void sensorsRefreshForScan(uint32_t nowMs);
uint8_t sensorsConsumeEvents();
