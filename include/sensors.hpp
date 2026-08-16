#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

void sensorsBegin();
void sensorsUpdate(uint32_t nowMs, bool motorsMoving, bool scanMode);
const            SensorData &sensorsGetData();

bool sensorsStartSoundWindow(uint32_t nowMs);
bool sensorsSoundWindowActive();
void sensorsRefreshForScan(uint32_t nowMs);

uint8_t sensorsConsumeEvents();
