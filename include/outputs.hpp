#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

void outputsBegin();
void outputsSetLightMode(LightMode mode);
LightMode outputsGetLightMode();
void outputsPulseAlarm(uint32_t nowMs, uint16_t durationMs);
void outputsUpdate(bool dark, bool warningActive, bool emergencyActive, uint32_t nowMs);
