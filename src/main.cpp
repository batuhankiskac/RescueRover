#include <Arduino.h>
#include "bluetooth.hpp"
#include "config.hpp"
#include "debug.hpp"
#include "motor.hpp"
#include "outputs.hpp"
#include "safety.hpp"
#include "sensors.hpp"
#include "telemetry.hpp"

namespace {
enum class ScanPhase : uint8_t {
    NONE, SETTLING, SOUND_WINDOW
};

uint32_t bootMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t scanPhaseStartMs = 0;
ScanPhase scanPhase = ScanPhase::NONE;
RobotState robotState = RobotState::STARTUP;
uint8_t selectedSpeed = DEFAULT_MOTOR_SPEED;

bool scanActive() {
    return scanPhase != ScanPhase::NONE;
}

RobotState calculateRobotState(uint32_t nowMs) {
    const SafetyStatus& safety = safetyGetStatus();
    if (safety.emergencyActive) {
        return RobotState::EMERGENCY;
    }
    if (static_cast<uint32_t>(nowMs - bootMs) < STARTUP_DURATION_MS) {
        return RobotState::STARTUP;
    }
    if (safety.warningActive) {
        return RobotState::WARNING;
    }
    if (scanActive()) {
        return RobotState::SCANNING;
    }
    if (motorIsMoving()) {
        return RobotState::DRIVING;
    }
    return RobotState::IDLE;
}

void beginScan(uint32_t nowMs) {
    stopMotors();
    scanPhase = ScanPhase::SETTLING;
    scanPhaseStartMs = nowMs;
}

void cancelScan() {
    scanPhase = ScanPhase::NONE;
}

void sendSafetyEvents(uint16_t events, uint32_t nowMs) {
    if ((events & SAFETY_EVENT_OBSTACLE) != 0U) {
        telemetrySendAlert(AlertCode::OBSTACLE);
    }
    if ((events & SAFETY_EVENT_GAS_WARNING) != 0U) {
        telemetrySendAlert(AlertCode::GAS_WARNING);
    }
    if ((events & SAFETY_EVENT_GAS_CRITICAL) != 0U) {
        telemetrySendAlert(AlertCode::GAS_CRITICAL);
    }
    if ((events & SAFETY_EVENT_TEMP_WARNING) != 0U) {
        telemetrySendAlert(AlertCode::TEMP_WARNING);
    }
    if ((events & SAFETY_EVENT_TEMP_CRITICAL) != 0U) {
        telemetrySendAlert(AlertCode::TEMP_CRITICAL);
    }
    if ((events & SAFETY_EVENT_TILT_WARNING) != 0U) {
        telemetrySendAlert(AlertCode::TILT_WARNING);
    }
    if ((events & SAFETY_EVENT_TILT_CRITICAL) != 0U) {
        telemetrySendAlert(AlertCode::TILT_CRITICAL);
    }
    if ((events & SAFETY_EVENT_COMMAND_TIMEOUT) != 0U) {
        telemetrySendAlert(AlertCode::COMMAND_TIMEOUT);
    }
    if ((events & SAFETY_EVENT_ULTRASONIC_FAULT) != 0U) {
        telemetrySendAlert(AlertCode::ULTRASONIC_FAULT);
    }
    if (events != SAFETY_EVENT_NONE) {
        outputsPulseAlarm(nowMs, ALERT_PULSE_MS);
    }
}

void sendSensorEvents(uint8_t events, uint32_t nowMs) {
    if ((events & SENSOR_EVENT_SOUND) != 0U) {
        telemetrySendAlert(AlertCode::SOUND_DETECTED);
    }
    if ((events & SENSOR_EVENT_MOTION) != 0U) {
        telemetrySendAlert(AlertCode::MOTION_DETECTED);
    }
    if (events != SENSOR_EVENT_NONE) {
        outputsPulseAlarm(nowMs, ALERT_PULSE_MS);
    }
}

bool executeMotion(MotionCommand command, uint32_t nowMs) {
    if (!safetyCanMove(command)) {
        telemetrySendAlert(command == MotionCommand::FORWARD ? AlertCode::FORWARD_BLOCKED : AlertCode::MOTION_BLOCKED);
        outputsPulseAlarm(nowMs, ALERT_PULSE_MS);
        return false;
    }

    bool accepted = false;
    switch (command) {
        case MotionCommand::FORWARD:
            accepted = moveForward(selectedSpeed);
            break;
        case MotionCommand::BACKWARD:
            accepted = moveBackward(selectedSpeed);
            break;
        case MotionCommand::LEFT:
            accepted = turnLeft(selectedSpeed);
            break;
        case MotionCommand::RIGHT:
            accepted = turnRight(selectedSpeed);
            break;
        case MotionCommand::STOP:
            stopMotors();
            accepted = true;
            break;
    }
    if (accepted) {
        cancelScan();
        safetyNoteControlContact(nowMs);
    } else {
        telemetrySendAlert(command == MotionCommand::FORWARD ? AlertCode::FORWARD_BLOCKED : AlertCode::MOTION_BLOCKED);
        outputsPulseAlarm(nowMs, ALERT_PULSE_MS);
    }
    return accepted;
}

bool isSpeedCommand(BluetoothCommand command) {
    return command >= BluetoothCommand::SPEED_0 && command <= BluetoothCommand::SPEED_9;
}

void processBluetooth(uint32_t nowMs) {
    const BluetoothCommand command = bluetoothPoll();
    if (command == BluetoothCommand::NONE) {
        return;
    }

    if (isSpeedCommand(command)) {
        const uint8_t digit = static_cast<uint8_t>(command) - static_cast<uint8_t>(BluetoothCommand::SPEED_0);
        selectedSpeed = static_cast<uint8_t>(MIN_MOTOR_SPEED + (static_cast<uint16_t>(255U - MIN_MOTOR_SPEED) * digit) / 9U);
        return;
    }

    switch (command) {
        case BluetoothCommand::FORWARD:
            (void)executeMotion(MotionCommand::FORWARD, nowMs);
            break;
        case BluetoothCommand::BACKWARD:
            (void)executeMotion(MotionCommand::BACKWARD, nowMs);
            break;
        case BluetoothCommand::LEFT:
            (void)executeMotion(MotionCommand::LEFT, nowMs);
            break;
        case BluetoothCommand::RIGHT:
            (void)executeMotion(MotionCommand::RIGHT, nowMs);
            break;
        case BluetoothCommand::STOP:
            (void)executeMotion(MotionCommand::STOP, nowMs);
            break;
        case BluetoothCommand::SCAN:
            beginScan(nowMs);
            break;
        case BluetoothCommand::EMERGENCY_STOP:
            cancelScan();
            safetyRequestEmergencyStop();
            telemetrySendAlert(AlertCode::EMERGENCY_STOP);
            break;
        case BluetoothCommand::CLEAR_EMERGENCY:
            if (safetyClearEmergency()) {
                telemetrySendAlert(AlertCode::EMERGENCY_CLEARED);
            } else {
                telemetrySendAlert(AlertCode::CLEAR_REJECTED);
            }
            break;
        case BluetoothCommand::HEARTBEAT:
            safetyNoteControlContact(nowMs);
            break;
        case BluetoothCommand::LIGHT_ON:
            outputsSetLightMode(LightMode::FORCED_ON);
            break;
        case BluetoothCommand::LIGHT_OFF:
            outputsSetLightMode(LightMode::FORCED_OFF);
            break;
        case BluetoothCommand::LIGHT_AUTO:
            outputsSetLightMode(LightMode::AUTO);
            break;
        case BluetoothCommand::HELP:
            bluetoothSendHelp();
            break;
        default:
            break;
    }
}

void updateScan(uint32_t nowMs) {
    if (scanPhase == ScanPhase::SETTLING && static_cast<uint32_t>(nowMs - scanPhaseStartMs) >= SCAN_SETTLE_MS) {
        (void)sensorsStartSoundWindow(nowMs);
        scanPhase = ScanPhase::SOUND_WINDOW;
        return;
    }

    if (scanPhase == ScanPhase::SOUND_WINDOW && !sensorsSoundWindowActive()) {
        sensorsRefreshForScan(nowMs);
        const uint16_t events = safetyUpdate(sensorsGetData(), nowMs);
        sendSafetyEvents(events, nowMs);
        robotState = calculateRobotState(nowMs);
        telemetrySendScan(sensorsGetData(), robotState);
        scanPhase = ScanPhase::NONE;
    }
}

}  // namespace

void setup() {
    DEBUG_BEGIN(USB_SERIAL_BAUD);
    motorBegin();
    outputsBegin();
    bluetoothBegin();
    sensorsBegin();

    bootMs = millis();
    lastTelemetryMs = bootMs;
    safetyBegin(bootMs);

    bluetoothOutput().println(F("RESCUE_ROVER:READY"));
    DEBUG_PRINTLN(F("RESCUE_ROVER:READY"));
}

void loop() {
    const uint32_t nowMs = millis();

    processBluetooth(nowMs);
    sensorsUpdate(nowMs, motorIsMoving(), scanActive());
    sendSensorEvents(sensorsConsumeEvents(), nowMs);
    sendSafetyEvents(safetyUpdate(sensorsGetData(), nowMs), nowMs);
    updateScan(nowMs);

    robotState = calculateRobotState(nowMs);
    const SafetyStatus& safety = safetyGetStatus();
    outputsUpdate(sensorsGetData().dark, safety.warningActive, safety.emergencyActive, nowMs);

    if (static_cast<uint32_t>(nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs = nowMs;
        telemetrySend(sensorsGetData(), robotState);
    }
}
