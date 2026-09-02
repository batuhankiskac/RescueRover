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

enum SafetyLevel : uint8_t {
    LEVEL_NORMAL,
    LEVEL_WARNING,
    LEVEL_ALERT,
    LEVEL_CRITICAL
};

struct HazardLevels {
    SafetyLevel obstacle;
    SafetyLevel gas;
    SafetyLevel temperature;
    SafetyLevel tilt;
};

Motion motion = MOTION_STOP;
ScanPhase scanPhase = SCAN_OFF;

uint32_t bootMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t scanStartedMs = 0;
uint32_t criticalBeepStartedMs = 0;
bool criticalBeepActive = false;
SafetyLevel pendingEventLevel = LEVEL_NORMAL;
const __FlashStringHelper* pendingEventMessage = NULL;

bool manualEmergency = false;
bool rolloverEmergency = false;
bool blockAllMotion = false;
bool blockForward = true;
SafetyLevel safetyLevel = LEVEL_NORMAL;

HazardLevels hazards = {};
HazardLevels previousHazards = {};
bool ultrasonicReady = false;
bool collisionCriticalAnnounced = false;
bool oldUltrasonicAlert = false;

bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
    return (uint32_t)(now - since) >= period;
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

void queueEvent(SafetyLevel level, const __FlashStringHelper* message) {
    if (pendingEventMessage == NULL || level > pendingEventLevel) {
        pendingEventLevel = level;
        pendingEventMessage = message;
    }
}

void flushEvent(uint32_t now) {
    if (pendingEventMessage == NULL) {
        return;
    }
    bool startBeep = pendingEventLevel == LEVEL_CRITICAL &&
                     (!criticalBeepActive ||
                      elapsed(now, criticalBeepStartedMs, CRITICAL_BEEP_MS));
    bluetooth.println(pendingEventMessage);
    if (startBeep) {
        criticalBeepActive = true;
        criticalBeepStartedMs = millis();
    }
    pendingEventLevel = LEVEL_NORMAL;
    pendingEventMessage = NULL;
}

void queueHazardRise(SafetyLevel current, SafetyLevel previous,
                     const __FlashStringHelper* warningMessage,
                     const __FlashStringHelper* criticalMessage) {
    if (current > previous) {
        queueEvent(current, current == LEVEL_CRITICAL ? criticalMessage : warningMessage);
    }
}

void signalCollisionRisk() {
    if (collisionCriticalAnnounced) {
        return;
    }
    collisionCriticalAnnounced = true;
    queueEvent(LEVEL_CRITICAL, F("CRITICAL:COLLISION_RISK"));
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

    if (startupActive(now) || !bluetoothConnected()) {
        stopMotors();
        return;
    }

    if (blockAllMotion) {
        queueEvent(LEVEL_ALERT, F("ALERT:MOTION_BLOCKED"));
        return;
    }

    if (blockForward && requested == MOTION_FORWARD) {
        if (hazards.obstacle == LEVEL_ALERT && !collisionCriticalAnnounced) {
            signalCollisionRisk();
        } else if (hazards.obstacle != LEVEL_ALERT) {
            queueEvent(LEVEL_ALERT, F("ALERT:FORWARD_BLOCKED"));
        }
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
            if (!manualEmergency) {
                manualEmergency = true;
                queueEvent(LEVEL_CRITICAL, F("CRITICAL:EMERGENCY_STOP"));
            }
            break;
        case 'C':
            if (hazards.gas == LEVEL_CRITICAL || hazards.tilt == LEVEL_CRITICAL ||
                hazards.temperature == LEVEL_CRITICAL ||
                (rolloverEmergency && !sensorsGetData().mpuValid)) {
                queueEvent(LEVEL_ALERT, F("ALERT:CLEAR_REJECTED"));
            } else {
                manualEmergency = false;
                rolloverEmergency = false;
                bluetooth.println(F("STATUS:EMERGENCY_CLEARED"));
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
    hazards.obstacle = !data.distanceValid ? LEVEL_NORMAL
                       : data.distanceCm <= OBSTACLE_CRITICAL_CM ? LEVEL_ALERT
                       : data.distanceCm <= OBSTACLE_WARNING_CM ? LEVEL_WARNING
                                                               : LEVEL_NORMAL;
    hazards.gas = data.gasState == GAS_CRITICAL ? LEVEL_CRITICAL
                  : data.gasState == GAS_WARNING ? LEVEL_WARNING
                                                 : LEVEL_NORMAL;
    hazards.temperature = data.temperatureCritical ? LEVEL_CRITICAL
                          : data.dhtValid && data.temperatureDeciC >= TEMP_WARNING_C * 10
                              ? LEVEL_WARNING
                              : LEVEL_NORMAL;

    int16_t pitch = abs(data.pitchDeg);
    int16_t roll = abs(data.rollDeg);
    int16_t greatestAngle = pitch > roll ? pitch : roll;
    if (data.mpuValid) {
        hazards.tilt = greatestAngle >= TILT_CRITICAL_DEG ? LEVEL_CRITICAL
                     : greatestAngle >= TILT_WARNING_DEG ? LEVEL_WARNING
                                                         : LEVEL_NORMAL;
    } else if (hazards.tilt != LEVEL_CRITICAL) {
        hazards.tilt = LEVEL_NORMAL;
    }
    bool newRollover = data.mpuValid && greatestAngle >= ROLLOVER_DEG &&
                       !rolloverEmergency;
    if (newRollover) {
        rolloverEmergency = true;
    }

    bool ultrasonicAlert = !startupActive(now) && !ultrasonicReady;
    if (hazards.obstacle == LEVEL_NORMAL) {
        collisionCriticalAnnounced = false;
    }

    if (hazards.obstacle == LEVEL_ALERT) {
        if (motion == MOTION_FORWARD) {
            signalCollisionRisk();
        } else if (previousHazards.obstacle != LEVEL_ALERT) {
            queueEvent(LEVEL_ALERT, F("ALERT:OBSTACLE_CLOSE"));
        }
    } else if (hazards.obstacle == LEVEL_WARNING &&
               previousHazards.obstacle == LEVEL_NORMAL) {
        queueEvent(LEVEL_WARNING, F("WARNING:OBSTACLE"));
    }
    queueHazardRise(hazards.gas, previousHazards.gas,
                    F("WARNING:GAS"), F("CRITICAL:GAS"));
    queueHazardRise(hazards.temperature, previousHazards.temperature,
                    F("WARNING:TEMPERATURE"), F("CRITICAL:TEMPERATURE"));
    if (newRollover) {
        queueEvent(LEVEL_CRITICAL, F("CRITICAL:ROLLOVER"));
    } else {
        queueHazardRise(hazards.tilt, previousHazards.tilt,
                        F("WARNING:TILT"), F("CRITICAL:TILT"));
    }
    if (ultrasonicAlert && !oldUltrasonicAlert) {
        queueEvent(LEVEL_ALERT, F("ALERT:ULTRASONIC_NOT_READY"));
    }
    previousHazards = hazards;
    oldUltrasonicAlert = ultrasonicAlert;

    blockAllMotion = manualEmergency || rolloverEmergency ||
                     hazards.gas == LEVEL_CRITICAL ||
                     hazards.temperature == LEVEL_CRITICAL ||
                     hazards.tilt == LEVEL_CRITICAL;
    blockForward = blockAllMotion || hazards.obstacle == LEVEL_ALERT ||
                   (!ultrasonicReady && ULTRASONIC_FAILSAFE_BLOCK_FORWARD);
    safetyLevel = blockAllMotion ? LEVEL_CRITICAL
                : hazards.obstacle == LEVEL_ALERT || ultrasonicAlert ? LEVEL_ALERT
                : hazards.obstacle == LEVEL_WARNING || hazards.gas == LEVEL_WARNING ||
                      hazards.temperature == LEVEL_WARNING ||
                      hazards.tilt == LEVEL_WARNING || data.pirMotion
                    ? LEVEL_WARNING
                    : LEVEL_NORMAL;

    if (startupActive(now) || !bluetoothConnected() || blockAllMotion ||
        (blockForward && motion == MOTION_FORWARD)) {
        stopMotors();
    }

    if (motion != MOTION_STOP && elapsed(now, lastControlMs, COMMAND_TIMEOUT_MS)) {
        stopMotors();
        queueEvent(LEVEL_ALERT, F("ALERT:COMMAND_TIMEOUT"));
    }
}

void sendSensorEvents() {
    uint8_t events = sensorsConsumeEvents();
    if (events & SENSOR_EVENT_SOUND) {
        queueEvent(LEVEL_ALERT, F("ALERT:SOUND_DETECTED"));
    } else if ((events & SENSOR_EVENT_MOTION) &&
               hazards.obstacle == LEVEL_NORMAL) {
        queueEvent(LEVEL_WARNING, F("WARNING:MOTION_DETECTED"));
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
        return F("LOCKED");
    }
    if (startupActive(now)) {
        return F("STARTUP");
    }
    if (scanPhase != SCAN_OFF) {
        return F("SCANNING");
    }
    if (motion != MOTION_STOP) {
        return F("DRIVING");
    }
    return F("IDLE");
}

const __FlashStringHelper* safetyLevelText(SafetyLevel level) {
    switch (level) {
        case LEVEL_WARNING:
            return F("WARNING");
        case LEVEL_ALERT:
            return F("ALERT");
        case LEVEL_CRITICAL:
            return F("CRITICAL");
        default:
            return F("NORMAL");
    }
}

void printDecimal(Print& output, int16_t value) {
    if (value < 0) {
        output.print('-');
        value = -value;
    }
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
        printDecimal(output, data.temperatureDeciC);
    } else {
        output.print(F("NA"));
    }
    output.print(F(",H:"));
    if (data.dhtValid) {
        printDecimal(output, static_cast<int16_t>(data.humidityDeciPct));
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
    output.print(robotStateText(now));
    output.print(F(",LEVEL:"));
    output.println(safetyLevelText(safetyLevel));
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
        printTelemetry(bluetooth, now, true);
        scanPhase = SCAN_OFF;
    }
}

void updateOutputs(uint32_t now) {
    const SensorData& data = sensorsGetData();
    writeOutput(Pins::HEADLIGHT, data.dark, HEADLIGHT_ACTIVE_HIGH);
    if (criticalBeepActive && elapsed(now, criticalBeepStartedMs, CRITICAL_BEEP_MS)) {
        criticalBeepActive = false;
    }
    writeOutput(Pins::ALARM, criticalBeepActive, BUZZER_ACTIVE_HIGH);
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
    updateSafety(now);
    updateScan(now);
    sendSensorEvents();
    flushEvent(millis());
    updateOutputs(millis());

    if (elapsed(now, lastTelemetryMs, TELEMETRY_PERIOD_MS)) {
        lastTelemetryMs = now;
        printTelemetry(bluetooth, now, false);
    }
}
