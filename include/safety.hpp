#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

void safetyBegin(uint32_t nowMs);
void safetyNoteControlContact(uint32_t nowMs);
void safetyRequestEmergencyStop();
bool safetyClearEmergency();

uint16_t safetyUpdate(const SensorData & data, uint32_t nowMs);
bool safetyCanMove(MotionCommand command);
const            SafetyStatus &safetyGetStatus();
