#pragma once

#include <Arduino.h>

enum GasState : uint8_t {
    GAS_WARMING,
    GAS_NORMAL,
    GAS_WARNING,
    GAS_CRITICAL
};

struct SensorData {
    uint16_t distanceCm;
    uint16_t gasRaw;
    uint16_t gasFiltered;
    uint16_t soundAmplitude;
    uint16_t lightRaw;
    int16_t temperatureDeciC;
    uint16_t humidityDeciPct;
    int16_t pitchDeg;
    int16_t rollDeg;
    GasState gasState;
    bool distanceValid;
    bool dhtValid;
    bool mpuValid;
    bool pirMotion;
    bool dark;
};

const uint8_t SENSOR_EVENT_SOUND = 1 << 0;
const uint8_t SENSOR_EVENT_MOTION = 1 << 1;

void sensorsBegin();
void sensorsUpdate(uint32_t nowMs, bool motorsMoving, bool scanMode);
const SensorData& sensorsGetData();

void sensorsStartSoundWindow(uint32_t nowMs);
bool sensorsSoundWindowActive();
void sensorsRefreshForScan(uint32_t nowMs);
uint8_t sensorsConsumeEvents();
