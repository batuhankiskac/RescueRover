#include "host_fake.hpp"

#include "pins.hpp"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

void setup();
void loop();

namespace {

constexpr uint32_t kBootReadyMs = 2001U;
constexpr uint32_t kReversalDeadTimeMs = 75U;

struct Result {
    bool passed = true;

    void expect(bool condition, const std::string &message) {
        if (!condition) {
            passed = false;
            std::cerr << "FAIL: " << message << '\n';
        }
    }
};

void tick(uint32_t nowMs) {
    host::setTime(nowMs);
    loop();
}

bool sawLine(const std::string &line) {
    const std::vector<std::string> &lines = host::serialLines();
    return std::find(lines.begin(), lines.end(), line) != lines.end();
}

bool sawLineAfter(const std::string &first, const std::string &second) {
    const std::vector<std::string> &lines = host::serialLines();
    const std::vector<std::string>::const_iterator firstIt =
        std::find(lines.begin(), lines.end(), first);
    if (firstIt == lines.end()) {
        return false;
    }
    return std::find(firstIt + 1, lines.end(), second) != lines.end();
}

void bootReady() {
    host::resetIo();
    host::setDigitalInput(Pins::BT_STATE, true);
    host::setAnalogDefault(Pins::MQ135_ANALOG, 250U);
    host::setAnalogDefault(Pins::SOUND_ANALOG, 0U);
    host::setAnalogDefault(Pins::LIGHT_ANALOG, 0U);
    setup();
    tick(0U);
    tick(90U);
    tick(180U);
    tick(kBootReadyMs);
    host::clearSerial();
    host::clearWrites();
}

bool reversalAndCancel() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "W must drive at the default PWM before reversal");

    host::clearWrites();
    host::pushBluetooth("S");
    tick(2101U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "W->S must hold PWM at zero during the 75 ms neutral interval");
    result.expect(host::digitalValue(Pins::MOTOR_LEFT_IN1) == LOW &&
                      host::digitalValue(Pins::MOTOR_LEFT_IN2) == LOW &&
                      host::digitalValue(Pins::MOTOR_RIGHT_IN1) == LOW &&
                      host::digitalValue(Pins::MOTOR_RIGHT_IN2) == LOW,
                  "W->S must neutralize both H-bridge sides before reversal");

    host::pushBluetooth(" ");
    tick(2110U);
    tick(2101U + kReversalDeadTimeMs);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "space during reversal dead-time must cancel the queued reversal");
    return result.passed;
}

bool reversalCompletesAfterDeadTime() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    host::pushBluetooth("S");
    tick(2101U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "W->S must enter the neutral interval");

    tick(2101U + kReversalDeadTimeMs - 1U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "reverse PWM must remain zero before 75 ms elapses");
    tick(2101U + kReversalDeadTimeMs);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "reverse PWM must resume when the 75 ms interval elapses");
    result.expect(host::digitalValue(Pins::MOTOR_LEFT_IN1) == LOW &&
                      host::digitalValue(Pins::MOTOR_LEFT_IN2) == HIGH &&
                      host::digitalValue(Pins::MOTOR_RIGHT_IN1) == HIGH &&
                      host::digitalValue(Pins::MOTOR_RIGHT_IN2) == LOW,
                  "the delayed target must apply the configured backward polarity");
    return result.passed;
}

bool noUnnecessaryDeadTimeAndPivotDetection() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "STOP->W must start without reversal dead-time");
    host::pushBluetooth("W");
    tick(2101U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "W->W must continue without reversal dead-time");

    host::pushBluetooth("A");
    tick(2102U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "W->A must detect the reversing right motor side");
    tick(2102U + kReversalDeadTimeMs);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "pivot target must start after its reversal dead-time");
    return result.passed;
}

bool reversalEmergencyCancel() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    host::pushBluetooth("S");
    tick(2101U);
    host::pushBluetooth("X");
    tick(2110U);
    host::pushBluetooth("C");
    tick(2111U);
    tick(2101U + kReversalDeadTimeMs);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "X during dead-time must cancel the target even after C clears it");

    return result.passed;
}

bool reversalDisconnectCancel() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    host::pushBluetooth("S");
    tick(2101U);
    host::setDigitalInput(Pins::BT_STATE, false);
    tick(2110U);
    host::setDigitalInput(Pins::BT_STATE, true);
    tick(2101U + kReversalDeadTimeMs);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "Bluetooth disconnect during dead-time must cancel the target");

    return result.passed;
}

bool reversalTimeoutCancel() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2100U);
    host::pushBluetooth("S");
    tick(2101U);
    tick(3101U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "command timeout must cancel a pending reversal before it starts");
    result.expect(sawLine("ALERT:COMMAND_TIMEOUT"),
                  "pending reversal timeout must retain the existing alert");

    return result.passed;
}

bool reversalCriticalSafetyCancel() {
    Result result;
    bootReady();

    host::setAnalogDefault(Pins::MQ135_ANALOG, 1000U);
    tick(60001U);
    host::pushBluetooth("W");
    tick(60002U);
    host::pushBluetooth("S");
    tick(60003U);
    tick(60301U);
    tick(60302U);
    result.expect(sawLine("CRITICAL:GAS"),
                  "the critical gas transition must be delivered");
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "a critical sensor transition must cancel a pending reversal");
    return result.passed;
}

bool eventPriorityAndDrain() {
    Result result;
    bootReady();

    host::setAnalogDefault(Pins::MQ135_ANALOG, 1000U);
    tick(60001U);
    host::pushBluetooth("X");
    tick(60301U);
    tick(60302U);

    result.expect(sawLineAfter("CRITICAL:EMERGENCY_STOP", "CRITICAL:GAS"),
                  "equal-severity critical events must preserve and drain GAS after X");
    tick(60551U);
    result.expect(host::digitalValue(Pins::ALARM) == LOW,
                  "simultaneous critical messages must share one non-extended beep");
    return result.passed;
}

bool criticalBeepRestartsAfterExpiry() {
    Result result;
    bootReady();

    host::setAnalogDefault(Pins::MQ135_ANALOG, 1000U);
    tick(60001U);
    host::pushBluetooth("X");
    tick(60002U);
    result.expect(host::digitalValue(Pins::ALARM) == HIGH,
                  "the first critical event must start a beep");

    // The next gas read crosses from warning to critical after the prior beep
    // duration, with no intervening output pass to clear the active flag.
    tick(60302U);
    result.expect(sawLine("CRITICAL:GAS"),
                  "the later gas critical transition must be delivered");
    result.expect(host::digitalValue(Pins::ALARM) == HIGH,
                  "a fresh critical batch after expiry must start a new beep");
    return result.passed;
}

bool simultaneousRolloverAndTiltDrain() {
    Result result;
    host::resetIo();
    host::setDigitalInput(Pins::BT_STATE, true);
    host::setAnalogDefault(Pins::MQ135_ANALOG, 250U);
    host::setMpuDefault({0.0F, 0.7071068F, 0.7071068F});
    setup();
    tick(0U);
    host::clearSerial();

    host::setMpuDefault({0.0F, 1.0F, 0.0F});
    tick(40U);
    tick(80U);
    tick(120U);
    tick(121U);
    result.expect(sawLineAfter("CRITICAL:ROLLOVER", "CRITICAL:TILT"),
                  "simultaneous rollover and confirmed tilt events must both drain");
    return result.passed;
}

bool soundAndPir() {
    Result result;
    bootReady();

    // Finish the automatic post-boot window before arming the deliberately
    // simultaneous window; otherwise the scripted samples belong to that old
    // window and sound/PIR arrive in separate loop passes.
    tick(2051U);
    host::setDigitalInput(Pins::PIR, true);
    host::setAnalogScript(Pins::SOUND_ANALOG, {0U, 100U, 0U, 100U});
    tick(29950U);
    tick(30000U);
    tick(30001U);

    result.expect(sawLine("ALERT:SOUND_DETECTED"),
                  "sound event must be reported");
    result.expect(sawLine("WARNING:MOTION_DETECTED"),
                  "PIR edge must drain after simultaneous sound without an obstacle");
    return result.passed;
}

bool soundPirObstacleSuppressed() {
    Result result;
    bootReady();

    tick(2051U);
    host::setPingCmScript({5U});
    tick(2100U);
    host::clearSerial();
    host::setDigitalInput(Pins::PIR, true);
    host::setAnalogScript(Pins::SOUND_ANALOG, {0U, 100U, 0U, 100U});
    tick(29950U);
    tick(30000U);
    tick(30001U);

    result.expect(sawLine("ALERT:SOUND_DETECTED"),
                  "sound event remains reportable near an obstacle");
    result.expect(!sawLine("WARNING:MOTION_DETECTED"),
                  "PIR event must be suppressed when obstacle context is active");
    return result.passed;
}

bool speedAckAndTelemetry() {
    Result result;
    bootReady();

    host::pushBluetooth("0");
    tick(2100U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "a speed digit must not start movement");
    host::pushBluetooth("W");
    tick(2101U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 90,
                  "digit 0 must set 90 PWM");
    result.expect(sawLine("STATUS:SPEED:90"),
                  "digit 0 must acknowledge STATUS:SPEED:90");

    host::pushBluetooth("9");
    tick(2102U);
    result.expect(sawLine("STATUS:SPEED:255"),
                  "digit 9 must acknowledge STATUS:SPEED:255");
    host::pushBluetooth("W");
    tick(2103U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 255,
                  "digit 9 must set 255 PWM");

    host::pushBluetooth("5");
    tick(2104U);
    result.expect(sawLine("STATUS:SPEED:181"),
                  "digit 5 must acknowledge STATUS:SPEED:181");
    host::pushBluetooth("W");
    tick(2105U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 181,
                  "digit 5 must set 181 PWM");
    tick(2601U);
    result.expect(std::any_of(host::serialLines().begin(), host::serialLines().end(),
                              [](const std::string &line) {
                                  return line.find(",SPD:181") != std::string::npos;
                              }),
                  "periodic telemetry must include SPD:181");
    return result.passed;
}

bool ultrasonicFailureAndRecovery() {
    Result result;
    bootReady();

    host::pushBluetooth("W");
    tick(2090U);
    host::setPingCmScript({0U, 0U, 0U, 50U, 50U, 50U});
    tick(2100U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "one temporary ultrasonic failure must not stop forward motion");
    tick(2190U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "two temporary ultrasonic failures must not stop forward motion");
    tick(2280U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "the third ultrasonic failure must stop active forward motion");
    host::pushBluetooth("W");
    tick(2281U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "three ultrasonic failures must block forward motion");
    host::pushBluetooth("S");
    tick(2282U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "ultrasonic unavailability must preserve backward escape motion");
    host::pushBluetooth(" ");
    tick(2283U);
    host::pushBluetooth("A");
    tick(2284U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "ultrasonic unavailability must preserve pivot escape motion");
    host::pushBluetooth(" ");
    tick(2285U);

    tick(2370U);
    host::pushBluetooth("W");
    tick(2371U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "first recovery sample must not unblock forward motion");
    tick(2460U);
    host::pushBluetooth("W");
    tick(2461U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "second recovery sample must not unblock forward motion");
    tick(2550U);
    host::pushBluetooth("W");
    tick(2551U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "third valid recovery sample must unblock forward motion");
    return result.passed;
}

bool dhtHysteresis() {
    Result result;
    host::resetIo();
    host::setDigitalInput(Pins::BT_STATE, true);
    host::setAnalogDefault(Pins::MQ135_ANALOG, 250U);
    host::setDhtScript({
        {true, 20.0F, 50.0F}, {true, 48.0F, 50.0F},
        {true, 48.0F, 50.0F}, {true, 47.0F, 50.0F},
        {true, 47.0F, 50.0F}, {false, 0.0F, 0.0F},
        {true, 44.0F, 50.0F}, {false, 0.0F, 0.0F},
        {true, 44.0F, 50.0F}, {true, 44.0F, 50.0F},
    });
    setup();
    tick(0U);
    tick(90U);
    tick(180U);
    tick(2001U);
    host::pushBluetooth("W");
    tick(2002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "one 48 C DHT sample must not lock motion");
    tick(4001U);
    host::pushBluetooth("W");
    tick(4002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "two 48 C DHT samples must lock motion");
    tick(6001U);
    tick(8001U);
    host::pushBluetooth("W");
    tick(8002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "valid readings from 45 through 47 C must retain the critical lock");
    tick(10001U);
    tick(12001U);
    host::pushBluetooth("W");
    tick(12002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "invalid DHT and one <45 C sample must retain the lock");
    tick(14001U);
    tick(16001U);
    host::pushBluetooth("W");
    tick(16002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "invalid DHT must reset the two-sample clear confirmation");
    tick(18001U);
    host::pushBluetooth("W");
    tick(18002U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "two consecutive <45 C DHT samples must clear the lock");
    return result.passed;
}

bool tiltHysteresis() {
    Result result;
    bootReady();

    host::setMpuDefault({0.0F, 1.0F, 0.0F});
    tick(2041U);
    tick(2081U);
    tick(2121U);
    host::pushBluetooth("W");
    tick(2122U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "first filtered >=50 degree tilt sample must not lock motion");
    tick(2161U);
    host::pushBluetooth("P");
    tick(2162U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "second filtered >=50 degree tilt sample must not lock motion");
    tick(2201U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "third filtered >=50 degree tilt sample must lock motion");

    host::setMpuDefault({0.0F, 0.0F, 1.0F});
    tick(2241U);
    tick(2281U);
    host::pushBluetooth("W");
    tick(2282U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "first filtered <45 degree tilt sample must retain the lock");
    tick(2321U);
    host::pushBluetooth("W");
    tick(2322U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 0,
                  "second filtered <45 degree tilt sample must retain the lock");
    tick(2361U);
    host::pushBluetooth("W");
    tick(2362U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "third filtered <45 degree tilt sample must clear the lock");
    return result.passed;
}

bool tiltInvalidRetentionAndRollover() {
    Result result;
    bootReady();

    host::setMpuDefault({0.0F, 1.0F, 0.0F});
    tick(2041U);
    tick(2081U);
    tick(2121U);
    tick(2161U);
    tick(2201U);
    tick(2241U);
    result.expect(sawLine("CRITICAL:ROLLOVER"),
                  "rollover must latch immediately when filtered angle reaches 70 degrees");

    host::setWireResult(1U);
    tick(2281U);
    host::pushBluetooth("C");
    tick(2282U);
    result.expect(sawLine("ALERT:CLEAR_REJECTED"),
                  "invalid MPU data must retain the critical/rollover lock");

    host::setWireResult(0U);
    host::setMpuDefault({0.0F, 0.0F, 1.0F});
    tick(2321U);
    host::pushBluetooth("C");
    tick(2322U);
    result.expect(sawLine("ALERT:CLEAR_REJECTED"),
                  "one valid upright MPU sample must not clear confirmed critical tilt");
    tick(2361U);
    tick(2401U);
    tick(2441U);
    tick(2481U);
    host::pushBluetooth("C");
    tick(2482U);
    result.expect(sawLine("STATUS:EMERGENCY_CLEARED"),
                  "confirmed safe MPU recovery plus C must clear rollover latch");
    host::pushBluetooth("W");
    tick(2483U);
    result.expect(host::analogValue(Pins::MOTOR_ENABLE_PWM) == 170,
                  "cleared rollover latch must permit motion");
    return result.passed;
}

struct Scenario {
    const char *name;
    bool (*run)();
};

const Scenario kScenarios[] = {
    {"reversal_and_cancel", reversalAndCancel},
    {"reversal_completes_after_deadtime", reversalCompletesAfterDeadTime},
    {"no_unnecessary_deadtime_and_pivot", noUnnecessaryDeadTimeAndPivotDetection},
    {"reversal_emergency_cancel", reversalEmergencyCancel},
    {"reversal_disconnect_cancel", reversalDisconnectCancel},
    {"reversal_timeout_cancel", reversalTimeoutCancel},
    {"reversal_critical_safety_cancel", reversalCriticalSafetyCancel},
    {"event_priority_and_drain", eventPriorityAndDrain},
    {"critical_beep_restarts_after_expiry", criticalBeepRestartsAfterExpiry},
    {"simultaneous_rollover_and_tilt", simultaneousRolloverAndTiltDrain},
    {"sound_and_pir", soundAndPir},
    {"sound_pir_obstacle_suppressed", soundPirObstacleSuppressed},
    {"speed_ack_and_telemetry", speedAckAndTelemetry},
    {"ultrasonic_failure_and_recovery", ultrasonicFailureAndRecovery},
    {"dht_hysteresis", dhtHysteresis},
    {"tilt_hysteresis", tiltHysteresis},
    {"tilt_invalid_retention_and_rollover", tiltInvalidRetentionAndRollover},
};

}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: rover_host <scenario>\n";
        return 2;
    }
    for (const Scenario &scenario : kScenarios) {
        if (std::string(argv[1]) == scenario.name) {
            const bool passed = scenario.run();
            std::cout << (passed ? "PASS: " : "FAIL: ") << scenario.name << '\n';
            return passed ? 0 : 1;
        }
    }
    std::cerr << "unknown scenario: " << argv[1] << '\n';
    return 2;
}
