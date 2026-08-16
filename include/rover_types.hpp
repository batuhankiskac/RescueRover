#pragma once

#include <Arduino.h>

enum class RobotState:uint8_t {
    STARTUP,
    IDLE,
    DRIVING,
    SCANNING,
    WARNING,
    EMERGENCY
};

enum class MotionCommand:uint8_t {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

enum class GasState:uint8_t {
    WARMING,
    NORMAL,
    WARNING,
    CRITICAL
};

enum class LightMode:uint8_t {
    AUTO,
    FORCED_ON,
    FORCED_OFF
};

enum class BluetoothCommand:uint8_t {
    NONE,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP,
    SCAN,
    EMERGENCY_STOP,
    CLEAR_EMERGENCY,
    HEARTBEAT,
    LIGHT_ON,
    LIGHT_OFF,
    LIGHT_AUTO,
    HELP,
    SPEED_0,
    SPEED_1,
    SPEED_2,
    SPEED_3,
    SPEED_4,
    SPEED_5,
    SPEED_6,
    SPEED_7,
    SPEED_8,
    SPEED_9
};

struct SensorData {
    uint16_t distanceCm;
    uint16_t gasRaw;
    uint16_t gasFiltered;
    uint16_t soundAmplitude;
    int16_t temperatureDeciC;
    uint16_t humidityDeciPct;
    int16_t pitchDeg;
    int16_t rollDeg;
    GasState gasState;
    bool distanceValid;
    bool dhtValid;
    bool mpuValid;
    bool soundDetected;
    bool pirMotion;
    bool pirReady;
    bool dark;
};

struct MotorStatus {
    MotionCommand motion;
    uint8_t speed;
};

struct SafetyStatus {
    bool obstacleWarning;
    bool obstacleCritical;
    bool gasWarning;
    bool gasCritical;
    bool temperatureWarning;
    bool temperatureCritical;
    bool tiltWarning;
    bool tiltCritical;
    bool ultrasonicFault;
    bool warningActive;
    bool emergencyActive;
    bool blockForward;
    bool blockAllMotion;
};

enum SafetyEvent:uint16_t {
    SAFETY_EVENT_NONE = 0,
    SAFETY_EVENT_OBSTACLE = 1U << 0,
    SAFETY_EVENT_GAS_WARNING = 1U << 1,
    SAFETY_EVENT_GAS_CRITICAL = 1U << 2,
    SAFETY_EVENT_TEMP_WARNING = 1U << 3,
    SAFETY_EVENT_TEMP_CRITICAL = 1U << 4,
    SAFETY_EVENT_TILT_WARNING = 1U << 5,
    SAFETY_EVENT_TILT_CRITICAL = 1U << 6,
    SAFETY_EVENT_COMMAND_TIMEOUT = 1U << 7,
    SAFETY_EVENT_ULTRASONIC_FAULT = 1U << 8
};

enum SensorEvent:uint8_t {
    SENSOR_EVENT_NONE = 0,
    SENSOR_EVENT_SOUND = 1U << 0,
    SENSOR_EVENT_MOTION = 1U << 1
};
