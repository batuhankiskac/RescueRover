#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

enum class AlertCode:uint8_t {
    OBSTACLE,
    GAS_WARNING,
    GAS_CRITICAL,
    TEMP_WARNING,
    TEMP_CRITICAL,
    SOUND_DETECTED,
    MOTION_DETECTED,
    TILT_WARNING,
    TILT_CRITICAL,
    COMMAND_TIMEOUT,
    EMERGENCY_STOP,
    EMERGENCY_CLEARED,
    CLEAR_REJECTED,
    FORWARD_BLOCKED,
    MOTION_BLOCKED,
    ULTRASONIC_FAULT
};

void telemetrySend(const SensorData & data, RobotState state);
void telemetrySendScan(const SensorData & data, RobotState state);
void telemetrySendAlert(AlertCode code);
