#include "safety.hpp"

#include "config.hpp"
#include "motor.hpp"

namespace {

    SafetyStatus status = {};
    uint32_t lastControlContactMs = 0;
    bool manualEmergencyLatched = false;
    bool rolloverLatched = false;

    bool previousObstacleWarning = false;
    bool previousGasWarning = false;
    bool previousGasCritical = false;
    bool previousTempWarning = false;
    bool previousTempCritical = false;
    bool previousTiltWarning = false;
    bool previousTiltCritical = false;
    bool previousUltrasonicFault = false;

    int16_t absoluteAngle(int16_t value) {
        return value < 0 ? static_cast < int16_t > (-value) : value;
    }

}

void safetyBegin(uint32_t nowMs) {
    lastControlContactMs = nowMs;
    status.blockForward = true;
    status.ultrasonicFault = true;
    status.warningActive = true;
    motorSetSafetyInhibit(false, true);
}

void safetyNoteControlContact(uint32_t nowMs) {
    lastControlContactMs = nowMs;
}

void safetyRequestEmergencyStop() {
    manualEmergencyLatched = true;
    motorSetSafetyInhibit(true, true);
    stopMotors();
}

bool safetyClearEmergency() {
    const bool temperatureStops =
    status.temperatureCritical && (TEMP_CRITICAL_STOPS_MOTORS != 0);
    if (status.gasCritical || status.tiltCritical || temperatureStops) {
        return false;
    }
    manualEmergencyLatched = false;
    rolloverLatched = false;
    return true;
}

uint16_t safetyUpdate(const SensorData & data, uint32_t nowMs) {
    uint16_t events = SAFETY_EVENT_NONE;

    status.ultrasonicFault = !data.distanceValid;
    status.obstacleWarning =
        data.distanceValid && data.distanceCm <= OBSTACLE_WARNING_CM;
    status.obstacleCritical =
        data.distanceValid && data.distanceCm <= OBSTACLE_CRITICAL_CM;
status.gasWarning = data.gasState == GasState::WARNING ||
data.gasState == GasState::CRITICAL;
status.gasCritical = data.gasState == GasState::CRITICAL;
    status.temperatureWarning =
        data.dhtValid && data.temperatureDeciC >= TEMP_WARNING_C * 10;
    status.temperatureCritical =
        data.dhtValid && data.temperatureDeciC >= TEMP_CRITICAL_C * 10;

    int16_t greatestAngle = 0;
    if (data.mpuValid) {
        const int16_t pitch = absoluteAngle(data.pitchDeg);
        const int16_t roll = absoluteAngle(data.rollDeg);
        greatestAngle = pitch > roll ? pitch : roll;
    }
    status.tiltWarning = data.mpuValid && greatestAngle >= TILT_WARNING_DEG;
    status.tiltCritical = data.mpuValid && greatestAngle >= TILT_CRITICAL_DEG;
    if (data.mpuValid && greatestAngle >= ROLLOVER_DEG) {
        rolloverLatched = true;
    }

    if (status.obstacleWarning && !previousObstacleWarning) {
        events |= SAFETY_EVENT_OBSTACLE;
    }
    if (status.gasCritical && !previousGasCritical) {
        events |= SAFETY_EVENT_GAS_CRITICAL;
    } else if (status.gasWarning && !previousGasWarning) {
        events |= SAFETY_EVENT_GAS_WARNING;
    }
    if (status.temperatureCritical && !previousTempCritical) {
        events |= SAFETY_EVENT_TEMP_CRITICAL;
    } else if (status.temperatureWarning && !previousTempWarning) {
        events |= SAFETY_EVENT_TEMP_WARNING;
    }
    if (status.tiltCritical && !previousTiltCritical) {
        events |= SAFETY_EVENT_TILT_CRITICAL;
    } else if (status.tiltWarning && !previousTiltWarning) {
        events |= SAFETY_EVENT_TILT_WARNING;
    }
    if (status.ultrasonicFault && !previousUltrasonicFault) {
        events |= SAFETY_EVENT_ULTRASONIC_FAULT;
    }

    previousObstacleWarning = status.obstacleWarning;
    previousGasWarning = status.gasWarning;
    previousGasCritical = status.gasCritical;
    previousTempWarning = status.temperatureWarning;
    previousTempCritical = status.temperatureCritical;
    previousTiltWarning = status.tiltWarning;
    previousTiltCritical = status.tiltCritical;
    previousUltrasonicFault = status.ultrasonicFault;

    const bool temperatureStops =
    status.temperatureCritical && (TEMP_CRITICAL_STOPS_MOTORS != 0);
    status.blockAllMotion = manualEmergencyLatched || rolloverLatched ||
        status.gasCritical || status.tiltCritical ||
        temperatureStops;
    status.blockForward = status.blockAllMotion || status.obstacleCritical ||
        (status.ultrasonicFault &&
         (ULTRASONIC_FAILSAFE_BLOCK_FORWARD != 0));
    status.emergencyActive = status.blockAllMotion;
    status.warningActive = status.emergencyActive || status.obstacleWarning ||
        status.gasWarning || status.temperatureWarning ||
        status.tiltWarning || status.ultrasonicFault;

    motorSetSafetyInhibit(status.blockAllMotion, status.blockForward);

    if (motorIsMoving() &&
        static_cast < uint32_t > (nowMs - lastControlContactMs) >=
        COMMAND_TIMEOUT_MS) {
        stopMotors();
        events |= SAFETY_EVENT_COMMAND_TIMEOUT;
    }

    return events;
}

bool safetyCanMove(MotionCommand command) {
if (command == MotionCommand::STOP) {
        return true;
    }
    if (status.blockAllMotion) {
        return false;
    }
return command != MotionCommand::FORWARD || !status.blockForward;
}

const            SafetyStatus & safetyGetStatus() {
    return status;
}
