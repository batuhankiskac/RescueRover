#include <Arduino.h>
#include <SoftwareSerial.h>

#include "config.hpp"
#include "pins.hpp"
#include "sensors.hpp"

SoftwareSerial bluetooth(Pins::BT_RX, Pins::BT_TX);

enum Motion : uint8_t {
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_LEFT,
    MOTION_RIGHT
};

enum ScanPhase : uint8_t {
    SCAN_OFF,
    SCAN_SETTLING,
    SCAN_SOUND
};

Motion motion = MOTION_STOP;
ScanPhase scanPhase = SCAN_OFF;

uint32_t bootMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t scanStartedMs = 0;
uint32_t alarmUntilMs = 0;

bool manualEmergency = false;
bool rolloverEmergency = false;
bool blockAllMotion = false;
bool blockForward = true;
bool warningActive = true;

bool obstacleWarning = false;
bool gasWarning = false;
bool gasCritical = false;
bool temperatureWarning = false;
bool temperatureCritical = false;
bool tiltWarning = false;
bool tiltCritical = false;
bool ultrasonicFault = true;
bool ultrasonicReady = false;

bool oldObstacleWarning = false;
bool oldGasWarning = false;
bool oldGasCritical = false;
bool oldTemperatureWarning = false;
bool oldTemperatureCritical = false;
bool oldTiltWarning = false;
bool oldTiltCritical = false;
bool oldUltrasonicFault = false;

bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
    return (uint32_t)(now - since) >= period;
}

bool before(uint32_t now, uint32_t end) {
    return (int32_t)(now - end) < 0;
}

bool startupActive(uint32_t now) {
    return !elapsed(now, bootMs, STARTUP_DURATION_MS);
}

bool bluetoothConnected() {
    return digitalRead(Pins::BT_STATE) ==
           (BT_STATE_ACTIVE_HIGH ? HIGH : LOW);
}

void writeOutput(uint8_t pin, bool on, bool activeHigh) {
    digitalWrite(pin, on == activeHigh ? HIGH : LOW);
}

void pulseAlarm(uint32_t now, uint16_t duration) {
    uint32_t requestedEnd = now + duration;
    if (!before(requestedEnd, alarmUntilMs)) {
        alarmUntilMs = requestedEnd;
    }
}

void sendAlert(const __FlashStringHelper* message) {
    bluetooth.println(message);
}

void writeMotorSide(uint8_t pin1, uint8_t pin2, int8_t direction, bool reversed) {
    if (reversed) {
        direction = -direction;
    }

    if (direction > 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else if (direction < 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

void stopMotors() {
    analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
    writeMotorSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2, 0, false);
    writeMotorSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2, 0, false);
    motion = MOTION_STOP;
}

void runMotors(Motion newMotion, int8_t left, int8_t right) {
    analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
    writeMotorSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2,
                   left, MOTOR_LEFT_REVERSED != 0);
    writeMotorSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2,
                   right, MOTOR_RIGHT_REVERSED != 0);
    analogWrite(Pins::MOTOR_ENABLE_PWM, DEFAULT_MOTOR_SPEED);
    motion = newMotion;
}

void moveRover(Motion requested, uint32_t now) {
    if (requested == MOTION_STOP) {
        stopMotors();
        scanPhase = SCAN_OFF;
        lastControlMs = now;
        return;
    }

    if (startupActive(now)) {
        stopMotors();
        return;
    }

    if (!bluetoothConnected()) {
        stopMotors();
        return;
    }

    if (blockAllMotion || (blockForward && requested == MOTION_FORWARD)) {
        sendAlert(requested == MOTION_FORWARD
                      ? F("ALERT:FORWARD_BLOCKED")
                      : F("ALERT:MOTION_BLOCKED"));
        pulseAlarm(now, ALERT_PULSE_MS);
        return;
    }

    if (requested == MOTION_FORWARD) {
        runMotors(requested, 1, 1);
    } else if (requested == MOTION_BACKWARD) {
        runMotors(requested, -1, -1);
    } else if (requested == MOTION_LEFT) {
        runMotors(requested, 1, -1);
    } else {
        runMotors(requested, -1, 1);
    }

    scanPhase = SCAN_OFF;
    lastControlMs = now;
}

void processBluetooth(uint32_t now) {
    if (bluetooth.available() == 0) {
        return;
    }

    char command = (char)bluetooth.read();
    if (command >= 'a' && command <= 'z') {
        command -= 'a' - 'A';
    }

    switch (command) {
        case 'W':
            moveRover(MOTION_FORWARD, now);
            break;
        case 'S':
            moveRover(MOTION_BACKWARD, now);
            break;
        case 'A':
            moveRover(MOTION_LEFT, now);
            break;
        case 'D':
            moveRover(MOTION_RIGHT, now);
            break;
        case ' ':
            moveRover(MOTION_STOP, now);
            break;
        case 'T':
            stopMotors();
            scanPhase = SCAN_SETTLING;
            scanStartedMs = now;
            break;
        case 'X':
            stopMotors();
            scanPhase = SCAN_OFF;
            manualEmergency = true;
            sendAlert(F("ALERT:EMERGENCY_STOP"));
            break;
        case 'C':
            if (gasCritical || tiltCritical ||
                (temperatureCritical && TEMP_CRITICAL_STOPS_MOTORS)) {
                sendAlert(F("ALERT:CLEAR_REJECTED"));
            } else {
                manualEmergency = false;
                rolloverEmergency = false;
                sendAlert(F("ALERT:EMERGENCY_CLEARED"));
            }
            break;
        case 'P':
            lastControlMs = now;
            break;
        case '?':
            bluetooth.println(F("CMD:W,S,A,D,SPACE,T,X,C,P,?"));
            break;
    }
}

void updateSafety(uint32_t now) {
    const SensorData& data = sensorsGetData();
    if (data.distanceValid) {
        ultrasonicReady = true;
    }
    ultrasonicFault = !ultrasonicReady;
    obstacleWarning = data.distanceValid && data.distanceCm <= OBSTACLE_WARNING_CM;
    bool obstacleCritical = data.distanceValid && data.distanceCm <= OBSTACLE_CRITICAL_CM;
    gasWarning = data.gasState == GAS_WARNING || data.gasState == GAS_CRITICAL;
    gasCritical = data.gasState == GAS_CRITICAL;
    temperatureWarning = data.dhtValid && data.temperatureDeciC >= TEMP_WARNING_C * 10;
    temperatureCritical = data.dhtValid && data.temperatureDeciC >= TEMP_CRITICAL_C * 10;

    int16_t pitch = abs(data.pitchDeg);
    int16_t roll = abs(data.rollDeg);
    int16_t greatestAngle = pitch > roll ? pitch : roll;
    tiltWarning = data.mpuValid && greatestAngle >= TILT_WARNING_DEG;
    tiltCritical = data.mpuValid && greatestAngle >= TILT_CRITICAL_DEG;
    if (data.mpuValid && greatestAngle >= ROLLOVER_DEG) {
        rolloverEmergency = true;
    }

    bool newAlert = false;
    if (obstacleWarning && !oldObstacleWarning) {
        sendAlert(F("ALERT:OBSTACLE"));
        newAlert = true;
    }
    if (gasCritical && !oldGasCritical) {
        sendAlert(F("ALERT:GAS_CRITICAL"));
        newAlert = true;
    } else if (gasWarning && !oldGasWarning) {
        sendAlert(F("ALERT:GAS_WARNING"));
        newAlert = true;
    }
    if (temperatureCritical && !oldTemperatureCritical) {
        sendAlert(F("ALERT:TEMPERATURE_CRITICAL"));
        newAlert = true;
    } else if (temperatureWarning && !oldTemperatureWarning) {
        sendAlert(F("ALERT:TEMPERATURE_WARNING"));
        newAlert = true;
    }
    if (tiltCritical && !oldTiltCritical) {
        sendAlert(F("ALERT:TILT_CRITICAL"));
        newAlert = true;
    } else if (tiltWarning && !oldTiltWarning) {
        sendAlert(F("ALERT:TILT_WARNING"));
        newAlert = true;
    }
    if (ultrasonicFault && !oldUltrasonicFault) {
        sendAlert(F("ALERT:ULTRASONIC_FAULT"));
        newAlert = true;
    }
    oldObstacleWarning = obstacleWarning;
    oldGasWarning = gasWarning;
    oldGasCritical = gasCritical;
    oldTemperatureWarning = temperatureWarning;
    oldTemperatureCritical = temperatureCritical;
    oldTiltWarning = tiltWarning;
    oldTiltCritical = tiltCritical;
    oldUltrasonicFault = ultrasonicFault;

    bool temperatureStops = temperatureCritical && TEMP_CRITICAL_STOPS_MOTORS;
    blockAllMotion = manualEmergency || rolloverEmergency || gasCritical ||
                     tiltCritical || temperatureStops;
    blockForward = blockAllMotion || obstacleCritical ||
                   (ultrasonicFault && ULTRASONIC_FAILSAFE_BLOCK_FORWARD);
    warningActive = blockAllMotion || obstacleWarning || gasWarning ||
                    temperatureWarning || tiltWarning || ultrasonicFault;

    if (startupActive(now) || !bluetoothConnected() || blockAllMotion ||
        (blockForward && motion == MOTION_FORWARD)) {
        stopMotors();
    }

    if (motion != MOTION_STOP && elapsed(now, lastControlMs, COMMAND_TIMEOUT_MS)) {
        stopMotors();
        sendAlert(F("ALERT:COMMAND_TIMEOUT"));
        newAlert = true;
    }

    if (newAlert) {
        pulseAlarm(now, ALERT_PULSE_MS);
    }
}

void sendSensorAlerts(uint32_t now) {
    uint8_t events = sensorsConsumeEvents();
    if (events & SENSOR_EVENT_SOUND) {
        sendAlert(F("ALERT:SOUND_DETECTED"));
    }
    if (events & SENSOR_EVENT_MOTION) {
        sendAlert(F("ALERT:MOTION_DETECTED"));
    }
    if (events != 0) {
        pulseAlarm(now, ALERT_PULSE_MS);
    }
}

const __FlashStringHelper* gasStateText(GasState state) {
    switch (state) {
        case GAS_NORMAL:
            return F("NORMAL");
        case GAS_WARNING:
            return F("WARNING");
        case GAS_CRITICAL:
            return F("CRITICAL");
        default:
            return F("WARM");
    }
}

const __FlashStringHelper* robotStateText(uint32_t now) {
    if (blockAllMotion) {
        return F("EMERGENCY");
    }
    if (!elapsed(now, bootMs, STARTUP_DURATION_MS)) {
        return F("STARTUP");
    }
    if (warningActive) {
        return F("WARNING");
    }
    if (scanPhase != SCAN_OFF) {
        return F("SCANNING");
    }
    if (motion != MOTION_STOP) {
        return F("DRIVING");
    }
    return F("IDLE");
}

void printSignedDecimal(Print& output, int16_t value) {
    if (value < 0) {
        output.print('-');
        value = -value;
    }
    output.print(value / 10);
    output.print('.');
    output.print(value % 10);
}

void printUnsignedDecimal(Print& output, uint16_t value) {
    output.print(value / 10);
    output.print('.');
    output.print(value % 10);
}

void printTelemetry(Print& output, uint32_t now, bool scan) {
    const SensorData& data = sensorsGetData();
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
        printSignedDecimal(output, data.temperatureDeciC);
    } else {
        output.print(F("NA"));
    }
    output.print(F(",H:"));
    if (data.dhtValid) {
        printUnsignedDecimal(output, data.humidityDeciPct);
    } else {
        output.print(F("NA"));
    }
    output.print(F(",G:"));
    output.print(data.gasFiltered);
    output.print(F(",GR:"));
    output.print(data.gasRaw);
    output.print(F(",GS:"));
    output.print(gasStateText(data.gasState));
    output.print(F(",SND:"));
    output.print(data.soundAmplitude);
    output.print(F(",LRAW:"));
    output.print(data.lightRaw);
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
    output.println(robotStateText(now));
}

void sendTelemetry(uint32_t now, bool scan = false) {
    printTelemetry(bluetooth, now, scan);
}

void updateScan(uint32_t now) {
    if (scanPhase == SCAN_SETTLING && elapsed(now, scanStartedMs, SCAN_SETTLE_MS)) {
        sensorsStartSoundWindow(now);
        scanPhase = SCAN_SOUND;
        return;
    }

    if (scanPhase == SCAN_SOUND && !sensorsSoundWindowActive()) {
        sensorsRefreshForScan(now);
        updateSafety(now);
        sendTelemetry(now, true);
        scanPhase = SCAN_OFF;
    }
}

void updateOutputs(uint32_t now) {
    const SensorData& data = sensorsGetData();
    bool headlightsOn = data.dark;
    writeOutput(Pins::HEADLIGHT, headlightsOn, HEADLIGHT_ACTIVE_HIGH);

    bool buzzerOn = false;
    if (blockAllMotion) {
        buzzerOn = ((now / EMERGENCY_BEEP_HALF_PERIOD_MS) & 1) == 0;
    } else if (before(now, alarmUntilMs)) {
        buzzerOn = true;
    } else if (warningActive) {
        buzzerOn = (now % WARNING_BEEP_PERIOD_MS) < WARNING_BEEP_ON_MS;
    }
    writeOutput(Pins::ALARM, buzzerOn, BUZZER_ACTIVE_HIGH);
}

void setup() {
    digitalWrite(Pins::MOTOR_ENABLE_PWM, LOW);
    digitalWrite(Pins::MOTOR_LEFT_IN1, LOW);
    digitalWrite(Pins::MOTOR_LEFT_IN2, LOW);
    digitalWrite(Pins::MOTOR_RIGHT_IN1, LOW);
    digitalWrite(Pins::MOTOR_RIGHT_IN2, LOW);
    pinMode(Pins::MOTOR_ENABLE_PWM, OUTPUT);
    pinMode(Pins::MOTOR_LEFT_IN1, OUTPUT);
    pinMode(Pins::MOTOR_LEFT_IN2, OUTPUT);
    pinMode(Pins::MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(Pins::MOTOR_RIGHT_IN2, OUTPUT);
    pinMode(Pins::BT_STATE, INPUT);

    bluetooth.begin(BT_BAUD_RATE);
    bluetooth.listen();

    pinMode(Pins::HEADLIGHT, OUTPUT);
    pinMode(Pins::ALARM, OUTPUT);
    stopMotors();
    writeOutput(Pins::HEADLIGHT, false, HEADLIGHT_ACTIVE_HIGH);
    writeOutput(Pins::ALARM, false, BUZZER_ACTIVE_HIGH);

    sensorsBegin();
    bootMs = millis();
    lastControlMs = bootMs;
    lastTelemetryMs = bootMs;

    bluetooth.println(F("RESCUE_ROVER:READY"));
}

void loop() {
    uint32_t now = millis();
    processBluetooth(now);
    sensorsUpdate(now, motion != MOTION_STOP, scanPhase != SCAN_OFF);
    sendSensorAlerts(now);
    updateSafety(now);
    updateScan(now);
    updateOutputs(now);

    if (elapsed(now, lastTelemetryMs, TELEMETRY_PERIOD_MS)) {
        lastTelemetryMs = now;
        sendTelemetry(now);
    }
}
