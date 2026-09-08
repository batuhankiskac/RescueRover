#include <Arduino.h>
#include <SoftwareSerial.h>

#include "config.hpp"
#include "pins.hpp"
#include "sensors.hpp"

SoftwareSerial bluetooth(Pins::BT_RX, Pins::BT_TX);

// User movement commands supported by the Bluetooth protocol. Keeping the
// symbolic names separate from ASCII letters lets motor logic stay readable.
enum Motion : uint8_t {
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_LEFT,
    MOTION_RIGHT
};

// Scan is a small state machine so settling and sound sampling stay non-blocking.
// The loop can still receive a stop or movement command in every phase.
enum ScanPhase : uint8_t {
    SCAN_OFF,
    SCAN_SETTLING,
    SCAN_SOUND
};

// The highest active safety level determines the reported system severity. The
// level is a report/output state; individual hazards remain in HazardLevels.
enum SafetyLevel : uint8_t {
    LEVEL_NORMAL,
    LEVEL_WARNING,
    LEVEL_ALERT,
    LEVEL_CRITICAL
};

// Event IDs are ordered first by severity and then by source. A fixed bitmask
// preserves simultaneous events without allocating a queue or message buffer.
enum EventId : uint8_t {
    EVENT_EMERGENCY_STOP,
    EVENT_ROLLOVER,
    EVENT_COLLISION_RISK,
    EVENT_GAS_CRITICAL,
    EVENT_TEMPERATURE_CRITICAL,
    EVENT_TILT_CRITICAL,
    EVENT_MOTION_BLOCKED,
    EVENT_FORWARD_BLOCKED,
    EVENT_CLEAR_REJECTED,
    EVENT_OBSTACLE_CLOSE,
    EVENT_ULTRASONIC_NOT_READY,
    EVENT_COMMAND_TIMEOUT,
    	EVENT_SOUND_DETECTED,
    EVENT_OBSTACLE_WARNING,
    EVENT_GAS_WARNING,
    EVENT_TEMPERATURE_WARNING,
    EVENT_TILT_WARNING,
    EVENT_MOTION_DETECTED,
    EVENT_COUNT
};
static_assert(EVENT_COUNT <= 32, "Event bitmask exceeds uint32_t capacity");

// One severity is tracked independently for each hazard source so a new hazard
// can generate an event without losing the state of the other sensors.
struct HazardLevels {
    SafetyLevel obstacle;
    SafetyLevel gas;
    SafetyLevel temperature;
    SafetyLevel tilt;
};

// The rover starts stopped; no motor command is assumed after reset.
Motion motion = MOTION_STOP;
// A scan starts off and is enabled only by the T command.
ScanPhase scanPhase = SCAN_OFF;
// The rover starts at the calibrated default; digits 0..9 can select a speed.
uint8_t motorSpeed = DEFAULT_MOTOR_SPEED;
// Logical directions remain separate from the physical pin polarity so either
// motor side can detect an actual forward/reverse change before it is applied.
// After a stop, they retain the last energized direction for the short guard
// window so an immediate opposite command cannot bypass reversal dead-time.
int8_t currentLeftDirection = 0;
int8_t currentRightDirection = 0;
Motion pendingMotion = MOTION_STOP;
int8_t pendingLeftDirection = 0;
int8_t pendingRightDirection = 0;
bool motorReversePending = false;
bool motorStopGuardActive = false;

// These timestamps implement startup, command-refresh, telemetry, scan, and
// beep deadlines without delay(), which keeps Bluetooth responsive.
uint32_t bootMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t scanStartedMs = 0;
uint32_t criticalBeepStartedMs = 0;
uint32_t motorReverseStartedMs = 0;
uint32_t motorStoppedMs = 0;
// The buzzer output is a short pulse, not a continuous hazard-state output.
bool criticalBeepActive = false;
// Repeated instances of one pending event coalesce, while distinct events from
// the same loop remain queued for deterministic delivery on later passes.
uint32_t pendingEventBits = 0;
bool criticalBatchBeeped = false;

// X and rollover are latched independently of transient sensor warnings.
bool manualEmergency = false;
bool rolloverEmergency = false;
bool blockAllMotion = false;
bool blockForward = true;
SafetyLevel safetyLevel = LEVEL_NORMAL;

// Current and previous hazard values allow rising-edge announcements.
HazardLevels hazards = {};
HazardLevels previousHazards = {};
bool ultrasonicReady = false;
bool collisionCriticalAnnounced = false;
bool oldUltrasonicAlert = false;

// Unsigned subtraction keeps elapsed-time checks valid across millis() wraparound.
bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
    return (uint32_t)(now - since) >= period;
}

bool startupActive(uint32_t now) {
    // Motion remains blocked until the startup interval has elapsed.
    return !elapsed(now, bootMs, STARTUP_DURATION_MS);
}

bool bluetoothConnected() {
    // Use the configured polarity because HC-05 breakout boards can expose
    // STATE with different active levels.
    return digitalRead(Pins::BT_STATE) ==
           (BT_STATE_ACTIVE_HIGH ? HIGH : LOW);
}

void writeOutput(uint8_t pin, bool on, bool activeHigh) {
    // Convert a logical request into the actual GPIO level for either polarity.
    digitalWrite(pin, on == activeHigh ? HIGH : LOW);
}

SafetyLevel eventLevel(EventId event) {
    if (event <= EVENT_TILT_CRITICAL) {
        return LEVEL_CRITICAL;
    }
    if (event <= EVENT_SOUND_DETECTED) {
        return LEVEL_ALERT;
    }
    return LEVEL_WARNING;
}

const __FlashStringHelper* eventMessage(EventId event) {
    switch (event) {
        case EVENT_EMERGENCY_STOP:
            return F("CRITICAL:EMERGENCY_STOP");
        case EVENT_ROLLOVER:
            return F("CRITICAL:ROLLOVER");
        case EVENT_COLLISION_RISK:
            return F("CRITICAL:COLLISION_RISK");
        case EVENT_GAS_CRITICAL:
            return F("CRITICAL:GAS");
        case EVENT_TEMPERATURE_CRITICAL:
            return F("CRITICAL:TEMPERATURE");
        case EVENT_TILT_CRITICAL:
            return F("CRITICAL:TILT");
        case EVENT_MOTION_BLOCKED:
            return F("ALERT:MOTION_BLOCKED");
        case EVENT_FORWARD_BLOCKED:
            return F("ALERT:FORWARD_BLOCKED");
        case EVENT_CLEAR_REJECTED:
            return F("ALERT:CLEAR_REJECTED");
        case EVENT_OBSTACLE_CLOSE:
            return F("ALERT:OBSTACLE_CLOSE");
        case EVENT_ULTRASONIC_NOT_READY:
            return F("ALERT:ULTRASONIC_NOT_READY");
        case EVENT_COMMAND_TIMEOUT:
            return F("ALERT:COMMAND_TIMEOUT");
        case EVENT_SOUND_DETECTED:
            return F("ALERT:SOUND_DETECTED");
        case EVENT_OBSTACLE_WARNING:
            return F("WARNING:OBSTACLE");
        case EVENT_GAS_WARNING:
            return F("WARNING:GAS");
        case EVENT_TEMPERATURE_WARNING:
            return F("WARNING:TEMPERATURE");
        case EVENT_TILT_WARNING:
            return F("WARNING:TILT");
        case EVENT_MOTION_DETECTED:
            return F("WARNING:MOTION_DETECTED");
        default:
            return F("");
    }
}

void queueEvent(EventId event) {
    pendingEventBits |= 1UL << event;
}

void flushEvent() {
    if (pendingEventBits == 0) {
        // No event means no Bluetooth traffic and no buzzer change.
        return;
    }

    uint8_t selected = 0;
    while ((pendingEventBits & (1UL << selected)) == 0) {
        ++selected;
    }
    const EventId event = static_cast<EventId>(selected);
    const SafetyLevel level = eventLevel(event);
    const bool critical = level == LEVEL_CRITICAL;
    const bool previousBeepExpired = criticalBeepActive &&
        elapsed(millis(), criticalBeepStartedMs, CRITICAL_BEEP_MS);
    const bool startBeep = critical && !criticalBatchBeeped &&
                           (!criticalBeepActive || previousBeepExpired);

    // Transmit first so the non-blocking beep duration starts after the slow
    // SoftwareSerial write rather than being shortened by it.
    bluetooth.println(eventMessage(event));
    pendingEventBits &= ~(1UL << selected);
    if (critical) {
        criticalBatchBeeped = true;
        if (startBeep) {
            criticalBeepActive = true;
            criticalBeepStartedMs = millis();
        }
    }

    const uint32_t criticalMask =
        (1UL << (static_cast<uint8_t>(EVENT_TILT_CRITICAL) + 1U)) - 1UL;
    if ((pendingEventBits & criticalMask) == 0) {
        criticalBatchBeeped = false;
    }
}

void queueHazardRise(SafetyLevel current, SafetyLevel previous,
                     EventId warningEvent, EventId criticalEvent) {
    if (current > previous) {
        // Announce only a severity increase; steady hazards do not spam the link.
        queueEvent(current == LEVEL_CRITICAL ? criticalEvent : warningEvent);
    }
}

void signalCollisionRisk() {
    if (collisionCriticalAnnounced) {
        // Repeated W presses near the same obstacle share one critical event.
        return;
    }
    collisionCriticalAnnounced = true;
    queueEvent(EVENT_COLLISION_RISK);
}

// Apply a direction correction without changing the requested rover motion.
// A zero direction disables both H-bridge inputs so the motor coasts safely.
void writeMotorSide(uint8_t pin1, uint8_t pin2, int8_t direction, bool reversed) {
    if (reversed) {
        // The physical motor may be mounted opposite to the logical rover axis.
        direction = -direction;
    }

    if (direction > 0) {
        // One polarity is the logical forward direction for this motor side.
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    } else if (direction < 0) {
        // Swapping the two inputs produces the logical reverse direction.
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    } else {
        // LOW/LOW removes the drive command before the PWM enable is removed.
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

bool motionActive() {
    return motion != MOTION_STOP || motorReversePending;
}

void cancelPendingMotion() {
    motorReversePending = false;
    pendingMotion = MOTION_STOP;
    pendingLeftDirection = 0;
    pendingRightDirection = 0;
}

void stopMotors(uint32_t now) {
    const bool wasActive = motionActive();
    // Disable PWM first, then clear both direction pairs, and finally publish
    // the stopped state used by scan, sound, and telemetry logic.
    analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
    writeMotorSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2, 0, false);
    writeMotorSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2, 0, false);
    if (wasActive &&
        (currentLeftDirection != 0 || currentRightDirection != 0)) {
        motorStoppedMs = now;
        motorStopGuardActive = true;
    }
    cancelPendingMotion();
    motion = MOTION_STOP;
}

void applyMotorMotion(Motion newMotion, int8_t left, int8_t right) {
    analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
    writeMotorSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2,
                   left, MOTOR_LEFT_REVERSED != 0);
    writeMotorSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2,
                   right, MOTOR_RIGHT_REVERSED != 0);
    analogWrite(Pins::MOTOR_ENABLE_PWM, motorSpeed);
    currentLeftDirection = left;
    currentRightDirection = right;
    motorStopGuardActive = false;
    cancelPendingMotion();
    motion = newMotion;
}

// The L298N uses one shared PWM value. If either side would reverse, both sides
// coast while the loop continues servicing commands, sensors, and safety.
void runMotors(Motion newMotion, int8_t left, int8_t right, uint32_t now) {
    if (motorStopGuardActive &&
        elapsed(now, motorStoppedMs, MOTOR_REVERSE_DEADTIME_MS)) {
        motorStopGuardActive = false;
        currentLeftDirection = 0;
        currentRightDirection = 0;
    }

    const bool reversing =
        (currentLeftDirection != 0 && left != currentLeftDirection) ||
        (currentRightDirection != 0 && right != currentRightDirection);
    if (!reversing) {
        applyMotorMotion(newMotion, left, right);
        return;
    }

    analogWrite(Pins::MOTOR_ENABLE_PWM, 0);
    writeMotorSide(Pins::MOTOR_LEFT_IN1, Pins::MOTOR_LEFT_IN2, 0, false);
    writeMotorSide(Pins::MOTOR_RIGHT_IN1, Pins::MOTOR_RIGHT_IN2, 0, false);
    motion = MOTION_STOP;
    if (!motorReversePending) {
        motorReverseStartedMs =
            motorStopGuardActive ? motorStoppedMs : now;
    }
    motorStopGuardActive = false;
    pendingMotion = newMotion;
    pendingLeftDirection = left;
    pendingRightDirection = right;
    motorReversePending = true;
}

void updatePendingMotion(uint32_t now) {
    if (!motorReversePending ||
        !elapsed(now, motorReverseStartedMs, MOTOR_REVERSE_DEADTIME_MS)) {
        return;
    }
    applyMotorMotion(pendingMotion, pendingLeftDirection, pendingRightDirection);
}

void moveRover(Motion requested, uint32_t now) {
    // Safety gates are checked before any non-stop command reaches the driver.
    if (requested == MOTION_STOP) {
        // Space is an immediate normal stop and also cancels a pending scan.
        stopMotors(now);
        scanPhase = SCAN_OFF;
        lastControlMs = now;
        return;
    }

    if (startupActive(now) || !bluetoothConnected()) {
        // A command cannot bypass the reset protection or a lost Bluetooth link.
        stopMotors(now);
        return;
    }

    if (blockAllMotion) {
        // Gas, temperature, tilt, rollover, and X emergency stop block every axis.
        if (motorReversePending) {
            stopMotors(now);
        }
        queueEvent(EVENT_MOTION_BLOCKED);
        return;
    }

    if (blockForward && requested == MOTION_FORWARD) {
        // Only forward is blocked by a front obstacle or an unvalidated sonar;
        // backward and pivot escape remain available unless all motion is locked.
        if (motorReversePending) {
            stopMotors(now);
        }
        if (hazards.obstacle == LEVEL_ALERT && !collisionCriticalAnnounced) {
            signalCollisionRisk();
        } else if (hazards.obstacle != LEVEL_ALERT) {
            queueEvent(EVENT_FORWARD_BLOCKED);
        }
        return;
    }

    if (requested == MOTION_FORWARD) {
        // Both sides use positive logical direction for forward travel.
        runMotors(requested, 1, 1, now);
    } else if (requested == MOTION_BACKWARD) {
        // Both sides reverse together for backward travel.
        runMotors(requested, -1, -1, now);
    } else if (requested == MOTION_LEFT) {
        // Opposite side directions create a pivot to the left.
        runMotors(requested, 1, -1, now);
    } else {
        // The remaining movement value is right pivot.
        runMotors(requested, -1, 1, now);
    }

    scanPhase = SCAN_OFF;
    lastControlMs = now;
}

void processBluetooth(uint32_t now) {
    if (bluetooth.available() == 0) {
        // Avoid reading an invalid byte when no command is waiting.
        return;
    }

    char command = (char)bluetooth.read();
    if (command >= 'a' && command <= 'z') {
        // Lowercase input follows the same protocol as uppercase input.
        command -= 'a' - 'A';
    }

    if (command >= '0' && command <= '9') {
        // Map 0..9 linearly from the safe minimum to full PWM. A stopped rover
        // stays stopped; active motion receives the new shared PWM immediately.
        uint8_t level = (uint8_t)(command - '0');
        motorSpeed = MIN_MOTOR_SPEED +
                     ((uint16_t)(255U - MIN_MOTOR_SPEED) * level) / 9U;
        if (motion != MOTION_STOP && !motorReversePending) {
            analogWrite(Pins::MOTOR_ENABLE_PWM, motorSpeed);
        }
        bluetooth.print(F("STATUS:SPEED:"));
        bluetooth.print(motorSpeed);
        bluetooth.println();
        return;
    }

    // Commands are case-insensitive; newline characters and unknown commands
    // are ignored by the switch below.
    switch (command) {
        case 'W':
            // WASD movement is intentionally limited to one byte per command.
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
            // Space gives the operator a fast, non-latched stop.
            moveRover(MOTION_STOP, now);
            break;
        case 'T':
            // Stop before scanning so motor vibration does not contaminate data.
            stopMotors(now);
            scanPhase = SCAN_SETTLING;
            scanStartedMs = now;
            break;
        case 'X':
            // X is latched and therefore cannot be cleared by another movement
            // command or by a transient sensor recovery.
            stopMotors(now);
            scanPhase = SCAN_OFF;
            if (!manualEmergency) {
                manualEmergency = true;
                queueEvent(EVENT_EMERGENCY_STOP);
            }
            break;
        case 'C': {
            // Clear is rejected while a critical source is still active or the
            // rollover reading is invalid or still unsafe.
            const SensorData& data = sensorsGetData();
            const int16_t pitch = abs(data.pitchDeg);
            const int16_t roll = abs(data.rollDeg);
            const int16_t greatestAngle = pitch > roll ? pitch : roll;
            const bool rolloverUnsafe = rolloverEmergency &&
                (!data.mpuValid || greatestAngle >= TILT_CRITICAL_DEG);
            if (hazards.gas == LEVEL_CRITICAL || hazards.tilt == LEVEL_CRITICAL ||
                hazards.temperature == LEVEL_CRITICAL ||
                rolloverUnsafe) {
                queueEvent(EVENT_CLEAR_REJECTED);
            } else {
                manualEmergency = false;
                rolloverEmergency = false;
                bluetooth.println(F("STATUS:EMERGENCY_CLEARED"));
            }
            break;
        }
        case 'P':
            // P is a heartbeat that refreshes the same timeout as movement.
            lastControlMs = now;
            break;
        case '?':
            // Keep help text synchronized with the accepted command switch.
            bluetooth.println(F("CMD:W,S,A,D,SPACE,T,X,C,P,0-9,?"));
            break;
    }
}

void updateSafety(uint32_t now) {
    // Derive hazard levels first, then compute motion locks and telemetry state.
    const SensorData& data = sensorsGetData();
    // The sensor layer tolerates temporary misses, marks repeated failures
    // unavailable, and requires consecutive valid readings before recovery.
    ultrasonicReady = data.distanceValid;
    // Invalid distance is handled separately by blockForward; it is not called
    // a nearby obstacle because no physical distance was measured.
    hazards.obstacle = !data.distanceValid ? LEVEL_NORMAL
                       : data.distanceCm <= OBSTACLE_CRITICAL_CM ? LEVEL_ALERT
                       : data.distanceCm <= OBSTACLE_WARNING_CM ? LEVEL_WARNING
                                                               : LEVEL_NORMAL;
    // Gas state already includes warm-up and filtering from sensors.cpp.
    hazards.gas = data.gasState == GAS_CRITICAL ? LEVEL_CRITICAL
                  : data.gasState == GAS_WARNING ? LEVEL_WARNING
                                                 : LEVEL_NORMAL;
    // The critical flag requires consecutive readings; a valid warning reading
    // below the critical latch remains only a warning.
    hazards.temperature = data.temperatureCritical ? LEVEL_CRITICAL
                          : data.dhtValid && data.temperatureDeciC >= TEMP_WARNING_C * 10
                              ? LEVEL_WARNING
                              : LEVEL_NORMAL;

    int16_t pitch = abs(data.pitchDeg);
    int16_t roll = abs(data.rollDeg);
    // Either axis can roll the chassis, so safety uses the greater magnitude.
    int16_t greatestAngle = pitch > roll ? pitch : roll;
    hazards.tilt = data.tiltCritical ? LEVEL_CRITICAL
                 : data.mpuValid && greatestAngle >= TILT_WARNING_DEG
                     ? LEVEL_WARNING
                     : LEVEL_NORMAL;
    bool newRollover = data.mpuValid && greatestAngle >= ROLLOVER_DEG &&
                       !rolloverEmergency;
    if (newRollover) {
        // Rollover remains latched until a safe C command, even if the rover is
        // placed upright again immediately.
        rolloverEmergency = true;
    }

    // Startup and runtime failure use the same unavailable-state alert edge.
    bool ultrasonicAlert = !startupActive(now) && !ultrasonicReady;
    if (hazards.obstacle == LEVEL_NORMAL) {
        // Leaving the close zone rearms the next collision-risk announcement.
        collisionCriticalAnnounced = false;
    }

    if (hazards.obstacle == LEVEL_ALERT) {
        if (motion == MOTION_FORWARD) {
            // A near obstacle while moving is more urgent than a static warning.
            signalCollisionRisk();
        } else if (previousHazards.obstacle != LEVEL_ALERT) {
            // A static close obstacle blocks forward but does not beep until W
            // is actually attempted or the rover is already advancing.
            queueEvent(EVENT_OBSTACLE_CLOSE);
        }
    } else if (hazards.obstacle == LEVEL_WARNING &&
               previousHazards.obstacle == LEVEL_NORMAL) {
        queueEvent(EVENT_OBSTACLE_WARNING);
    }
    queueHazardRise(hazards.gas, previousHazards.gas,
                    EVENT_GAS_WARNING, EVENT_GAS_CRITICAL);
    queueHazardRise(hazards.temperature, previousHazards.temperature,
                    EVENT_TEMPERATURE_WARNING, EVENT_TEMPERATURE_CRITICAL);
    if (newRollover) {
        queueEvent(EVENT_ROLLOVER);
    }
    queueHazardRise(hazards.tilt, previousHazards.tilt,
                    EVENT_TILT_WARNING, EVENT_TILT_CRITICAL);
    if (ultrasonicAlert && !oldUltrasonicAlert) {
        // Report the startup validation failure once when it first appears.
        queueEvent(EVENT_ULTRASONIC_NOT_READY);
    }
    previousHazards = hazards;
    oldUltrasonicAlert = ultrasonicAlert;

    blockAllMotion = manualEmergency || rolloverEmergency ||
                     hazards.gas == LEVEL_CRITICAL ||
                     hazards.temperature == LEVEL_CRITICAL ||
                     hazards.tilt == LEVEL_CRITICAL;
    // Forward-only blocking preserves a reverse or pivot escape path when the
    // front sonar sees an obstacle, while critical hazards still block all axes.
    blockForward = blockAllMotion || hazards.obstacle == LEVEL_ALERT ||
                   (!ultrasonicReady && ULTRASONIC_FAILSAFE_BLOCK_FORWARD);
    safetyLevel = blockAllMotion ? LEVEL_CRITICAL
                : hazards.obstacle == LEVEL_ALERT || ultrasonicAlert ? LEVEL_ALERT
                : hazards.obstacle == LEVEL_WARNING || hazards.gas == LEVEL_WARNING ||
                      hazards.temperature == LEVEL_WARNING ||
                      hazards.tilt == LEVEL_WARNING || data.pirMotion
                    ? LEVEL_WARNING
                    : LEVEL_NORMAL;

    // Apply the computed policy immediately so a newly detected hazard cannot
    // leave the motors running until the next command arrives.
    const bool forwardActive = motion == MOTION_FORWARD ||
        (motorReversePending && pendingMotion == MOTION_FORWARD);
    if (startupActive(now) || !bluetoothConnected() || blockAllMotion ||
        (blockForward && forwardActive)) {
        stopMotors(now);
    }

    if (motionActive() && elapsed(now, lastControlMs, COMMAND_TIMEOUT_MS)) {
        // Missing heartbeats stop motion even if Bluetooth remains electrically
        // connected, protecting against a silent terminal or dropped command.
        stopMotors(now);
        queueEvent(EVENT_COMMAND_TIMEOUT);
    }
}

void sendSensorEvents() {
    uint8_t events = sensorsConsumeEvents();
    if (events & SENSOR_EVENT_SOUND) {
        // Sound is an operator-facing alert, not a reason to start the buzzer.
        queueEvent(EVENT_SOUND_DETECTED);
    }
    if ((events & SENSOR_EVENT_MOTION) &&
        hazards.obstacle == LEVEL_NORMAL) {
        // A nearby obstacle already explains the motion context, so suppress a
        // duplicate PIR warning while retaining PIR telemetry.
        queueEvent(EVENT_MOTION_DETECTED);
    }
}

// Keep text conversion in one place so telemetry remains compact and stable.
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
    if (motionActive()) {
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
    // Store one decimal place as an integer, then add the separator only when
    // formatting for the Bluetooth terminal.
    if (value < 0) {
        output.print('-');
        value = -value;
    }
    output.print(value / 10);
    output.print('.');
    output.print(value % 10);
}

void printTelemetry(Print& output, uint32_t now, bool scan) {
    // Telemetry is written directly to the Bluetooth stream to avoid a buffer.
    const SensorData& data = sensorsGetData();
    if (scan) {
        output.print(F("SCAN:"));
    }

    output.print(F("D:"));
    if (data.distanceValid) {
        // NA distinguishes a missing/invalid reading from a numeric distance.
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
    // G is filtered gas ADC, GR is raw ADC, and GS is its interpreted state.
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
    // LIGHT is a boolean interpretation; LRAW above preserves the calibration
    // evidence needed to tune the threshold later.
    output.print(data.dark ? 1 : 0);
    output.print(F(",SPD:"));
    output.print(motorSpeed);
    output.print(F(",STATE:"));
    output.print(robotStateText(now));
    output.print(F(",LEVEL:"));
    output.println(safetyLevelText(safetyLevel));
}

void updateScan(uint32_t now) {
    if (scanPhase == SCAN_SETTLING && elapsed(now, scanStartedMs, SCAN_SETTLE_MS)) {
        // Start acoustic sampling only after the quiet settling period.
        sensorsStartSoundWindow(now);
        scanPhase = SCAN_SOUND;
        return;
    }

    if (scanPhase == SCAN_SOUND && !sensorsSoundWindowActive()) {
        // Refresh all relevant sensors after the sound window, then emit one
        // SCAN line containing the synchronized operator snapshot.
        sensorsRefreshForScan(now);
        updateSafety(now);
        printTelemetry(bluetooth, now, true);
        scanPhase = SCAN_OFF;
    }
}

void updateOutputs(uint32_t now) {
    const SensorData& data = sensorsGetData();
    // Headlights follow the sensor continuously; they are not a manual command.
    writeOutput(Pins::HEADLIGHT, data.dark, HEADLIGHT_ACTIVE_HIGH);
    if (criticalBeepActive && elapsed(now, criticalBeepStartedMs, CRITICAL_BEEP_MS)) {
        // Clearing the flag here makes the beep duration non-blocking.
        criticalBeepActive = false;
    }
    writeOutput(Pins::ALARM, criticalBeepActive, BUZZER_ACTIVE_HIGH);
}

void setup() {
    // Preload motor outputs low before enabling their pin modes.
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

    // Start the serial link before announcing READY so the first message is not
    // lost while the HC-05 is being selected as the active listener.
    bluetooth.begin(BT_BAUD_RATE);
    bluetooth.listen();

    pinMode(Pins::HEADLIGHT, OUTPUT);
    pinMode(Pins::ALARM, OUTPUT);
    stopMotors(bootMs);
    writeOutput(Pins::HEADLIGHT, false, HEADLIGHT_ACTIVE_HIGH);
    writeOutput(Pins::ALARM, false, BUZZER_ACTIVE_HIGH);

    sensorsBegin();
    // All timeouts start from the same boot reference after initialization.
    bootMs = millis();
    lastControlMs = bootMs;
    lastTelemetryMs = bootMs;

    bluetooth.println(F("RESCUE_ROVER:READY"));
}

void loop() {
    // Keep command handling, sensing, safety, outputs, and telemetry in one
    // short cooperative cycle.
    uint32_t now = millis();
    // Read one command before sensing so an operator stop is handled promptly.
    processBluetooth(now);
    // Refresh scheduled sensors with the current movement and scan state.
    sensorsUpdate(now, motionActive(), scanPhase != SCAN_OFF);
    // Safety runs after fresh data and before output/telemetry decisions.
    updateSafety(now);
    // A reversal target can start only after the fresh safety pass accepts it.
    updatePendingMotion(now);
    updateScan(now);
    sendSensorEvents();
    flushEvent();
    updateOutputs(millis());

    if (elapsed(now, lastTelemetryMs, TELEMETRY_PERIOD_MS)) {
        lastTelemetryMs = now;
        printTelemetry(bluetooth, now, false);
    }
}
