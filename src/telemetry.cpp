#include "telemetry.hpp"

#include "bluetooth.hpp"
#include "config.hpp"

namespace {

    void printSignedFixed1(Print & output, int16_t value) {
        if (value < 0) {
            output.print('-');
            value = static_cast < int16_t > (-value);
        }
        output.print(value / 10);
        output.print('.');
        output.print(value % 10);
    }

    void printUnsignedFixed1(Print & output, uint16_t value) {
        output.print(value / 10U);
        output.print('.');
        output.print(value % 10U);
    }

    void printGasState(Print & output, GasState state) {
        switch (state) {
            case GasState::WARMING:output.print(F("WARM"));
            break;
            case GasState::NORMAL:output.print(F("NORMAL"));
            break;
            case GasState::WARNING:output.print(F("WARNING"));
            break;
            case GasState::CRITICAL:output.print(F("CRITICAL"));
            break;
        }
    }

    void printRobotState(Print & output, RobotState state) {
        switch (state) {
            case RobotState::STARTUP:output.print(F("STARTUP"));
            break;
            case RobotState::IDLE:output.print(F("IDLE"));
            break;
            case RobotState::DRIVING:output.print(F("DRIVING"));
            break;
            case RobotState::SCANNING:output.print(F("SCANNING"));
            break;
            case RobotState::WARNING:output.print(F("WARNING"));
            break;
            case RobotState::EMERGENCY:output.print(F("EMERGENCY"));
            break;
        }
    }

    void printPayload(Print & output, const SensorData & data, RobotState state, bool scan) {
        if (scan) {
            output.print(F("SCAN:"));
        }
        output.print(F("D:"));
        if (data.distanceValid) {
            output.print(data.distanceCm);
        } else {
            output.print(F("NA"));
        }
        output.print(F(",T:"));
        if (data.dhtValid) {
            printSignedFixed1(output, data.temperatureDeciC);
        } else {
            output.print(F("NA"));
        }
        output.print(F(",H:"));
        if (data.dhtValid) {
            printUnsignedFixed1(output, data.humidityDeciPct);
        } else {
            output.print(F("NA"));
        }
        output.print(F(",G:"));
        output.print(data.gasFiltered);
        output.print(F(",GR:"));
        output.print(data.gasRaw);
        output.print(F(",GS:"));
        printGasState(output, data.gasState);
        output.print(F(",SND:"));
        output.print(data.soundAmplitude);
        output.print(F(",PIR:"));
        output.print(data.pirMotion ? 1 : 0);
        output.print(F(",PITCH:"));
        if (data.mpuValid) {
            output.print(data.pitchDeg);
        } else {
            output.print(F("NA"));
        }
        output.print(F(",ROLL:"));
        if (data.mpuValid) {
            output.print(data.rollDeg);
        } else {
            output.print(F("NA"));
        }
        output.print(F(",LIGHT:"));
        output.print(data.dark ? 1 : 0);
        output.print(F(",STATE:"));
        printRobotState(output, state);
        output.println();
    }

const __FlashStringHelper *alertText(AlertCode code) {
        switch (code) {
            case AlertCode::OBSTACLE:return F("ALERT:OBSTACLE");
            case AlertCode::GAS_WARNING:return F("ALERT:GAS_WARNING");
            case AlertCode::GAS_CRITICAL:return F("ALERT:GAS_CRITICAL");
            case AlertCode::TEMP_WARNING:return F("ALERT:TEMPERATURE_WARNING");
            case AlertCode::TEMP_CRITICAL:return F("ALERT:TEMPERATURE_CRITICAL");
            case AlertCode::SOUND_DETECTED:return F("ALERT:SOUND_DETECTED");
            case AlertCode::MOTION_DETECTED:return F("ALERT:MOTION_DETECTED");
            case AlertCode::TILT_WARNING:return F("ALERT:TILT_WARNING");
            case AlertCode::TILT_CRITICAL:return F("ALERT:TILT_CRITICAL");
            case AlertCode::COMMAND_TIMEOUT:return F("ALERT:COMMAND_TIMEOUT");
            case AlertCode::EMERGENCY_STOP:return F("ALERT:EMERGENCY_STOP");
            case AlertCode::EMERGENCY_CLEARED:return F("ALERT:EMERGENCY_CLEARED");
            case AlertCode::CLEAR_REJECTED:return F("ALERT:CLEAR_REJECTED");
            case AlertCode::FORWARD_BLOCKED:return F("ALERT:FORWARD_BLOCKED");
            case AlertCode::MOTION_BLOCKED:return F("ALERT:MOTION_BLOCKED");
            case AlertCode::ULTRASONIC_FAULT:return F("ALERT:ULTRASONIC_FAULT");
        }
        return F("ALERT:UNKNOWN");
    }

}

void telemetrySend(const SensorData & data, RobotState state) {
    printPayload(bluetoothOutput(), data, state, false);
#if DEBUG_ENABLED
    printPayload(Serial, data, state, false);
#endif
}

void telemetrySendScan(const SensorData & data, RobotState state) {
    printPayload(bluetoothOutput(), data, state, true);
#if DEBUG_ENABLED
    printPayload(Serial, data, state, true);
#endif
}

void telemetrySendAlert(AlertCode code) {
    const __FlashStringHelper *message = alertText(code);
    bluetoothOutput().println(message);
#if DEBUG_ENABLED
    Serial.println(message);
#endif
}
