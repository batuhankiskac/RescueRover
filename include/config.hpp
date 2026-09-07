#pragma once

// The HC-05 data-mode default is 9600 baud, so both ends interpret each byte
// at the same rate without requiring a runtime configuration command.
#define BT_BAUD_RATE 9600UL
// Most HC-05 breakout boards drive STATE high while a link is connected.
#define BT_STATE_ACTIVE_HIGH 1

// Motion is rejected briefly after reset while the motor outputs and sensors
// settle. This also prevents a stale command from moving the rover at boot.
#define STARTUP_DURATION_MS 2000UL
// A direction command must be refreshed regularly; otherwise the rover stops.
#define COMMAND_TIMEOUT_MS 1000UL
// 500 ms gives the operator regular feedback without filling the slow
// 9600-baud Bluetooth link with continuous output.
#define TELEMETRY_PERIOD_MS 500UL
// The default motor speed remains the calibrated starting speed. Digits 0..9
// may select a temporary runtime speed after startup.
#define DEFAULT_MOTOR_SPEED 170U
#define MIN_MOTOR_SPEED 90U
#define MOTOR_REVERSE_DEADTIME_MS 75UL

// The assembled motor polarity is not identical on both sides, so the right
// side is inverted in software instead of changing the movement commands.
#define MOTOR_LEFT_REVERSED 0
#define MOTOR_RIGHT_REVERSED 1

// Sensor reads run at separate rates because DHT11 and I2C/ultrasonic reads
// have different timing costs and do not need to run on every loop.
#define MPU_PERIOD_MS 40UL
#define ULTRASONIC_PERIOD_MS 90UL
#define GAS_PERIOD_MS 300UL
#define LIGHT_PERIOD_MS 250UL
#define DHT_PERIOD_MS 2000UL
#define PIR_STARTUP_MS 30000UL

// Ignore impossible echo distances and require three valid startup samples
// before forward motion is trusted. The two limits map distance to severity.
#define ULTRASONIC_MIN_CM 2U
#define ULTRASONIC_MAX_CM 400U
#define ULTRASONIC_VALID_READINGS_REQUIRED 3U
#define ULTRASONIC_FAILURE_LIMIT 3U
#define ULTRASONIC_FAILSAFE_BLOCK_FORWARD 1
#define OBSTACLE_WARNING_CM 30U
#define OBSTACLE_CRITICAL_CM 10U

// A warning is reported at 45 C. Critical entry and recovery use separate
// thresholds so two confirmed readings cannot oscillate around one boundary.
#define TEMP_WARNING_C 45
#define TEMP_CRITICAL_C 48
#define TEMP_CRITICAL_CLEAR_C 45
#define TEMP_CRITICAL_READINGS_REQUIRED 2U

// MQ-135 values are ADC readings. The sensor first warms up, then its filtered
// value is compared with a baseline plus the warning or critical offset.
#define GAS_WARMUP_MS 60000UL
#define GAS_BASELINE_ADC 250U
#define GAS_WARNING_ABOVE_BASELINE 80U
#define GAS_CRITICAL_ABOVE_BASELINE 200U
#define GAS_FILTER_DIVISOR 4U

// A sound window records the minimum and maximum ADC values. Their difference
// is the amplitude used for the threshold; it is not a calibrated dB value.
#define SOUND_SAMPLE_WINDOW_MS 50UL
#define SOUND_SAMPLE_INTERVAL_US 250UL
#define SOUND_IDLE_PERIOD_MS 1000UL
#define SOUND_THRESHOLD 90U

// The MPU6050 normally answers at 0x68; the implementation also tries 0x69.
// Offsets correct mounting bias, while warning/critical/rollover define the
// safety response to the largest absolute pitch or roll angle.
#define MPU6050_ADDRESS 0x68U
#define MPU_PITCH_OFFSET_DEG 0
#define MPU_ROLL_OFFSET_DEG 0
#define TILT_WARNING_DEG 30
#define TILT_CRITICAL_DEG 50
#define TILT_CRITICAL_CLEAR_DEG 45
#define TILT_CRITICAL_READINGS_REQUIRED 3U
#define ROLLOVER_DEG 70

// The raw light threshold is interpreted according to the observed module
// polarity. Current hardware increases its ADC value in darkness.
#define LIGHT_DARK_THRESHOLD 500U
#define LIGHT_ANALOG_DARK_BELOW 0
#define HEADLIGHT_ACTIVE_HIGH 1

// These settings allow the same output helper to support either active-high or
// active-low hardware. Critical events use a short, non-blocking beep.
#define BUZZER_ACTIVE_HIGH 1
#define CRITICAL_BEEP_MS 250UL

// The rover remains still for this period so motor vibration does not dominate
// the sound and sensor values collected by the operator's scan command.
#define SCAN_SETTLE_MS 500UL
