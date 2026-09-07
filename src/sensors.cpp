#include "sensors.hpp"

#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <NewPing.h>
#include <Wire.h>
#include <math.h>
#include "config.hpp"
#include "pins.hpp"

namespace {

// Each library object is kept in this module so the main control loop only
// works with the compact SensorData structure. This keeps hardware-library
// details out of the Bluetooth and safety code.
DHT dht(Pins::DHT11_DATA, DHT11);
Adafruit_MPU6050 mpu;
NewPing sonar(Pins::ULTRASONIC_TRIG, Pins::ULTRASONIC_ECHO,
              ULTRASONIC_MAX_CM);

// Zero initialization gives the telemetry path defined values before the first
// scheduled read completes; the separate validity flags prevent them being
// presented as valid measurements.
SensorData data = {};
// Events are stored until the main loop converts them into severity messages.
uint8_t pendingEvents = 0;
// This timestamp starts warm-up and PIR stabilization from the same reference.
uint32_t sensorStartMs = 0;

// Each last-* timestamp prevents a slow sensor from running on every loop.
uint32_t lastMpuMs = 0;
uint32_t lastDistanceMs = 0;
uint32_t lastGasMs = 0;
uint32_t lastLightMs = 0;
uint32_t lastDhtMs = 0;
uint32_t lastSoundWindowMs = 0;

// The distance filter needs a small fixed buffer because dynamic allocation is
// undesirable on the Uno. The index wraps after the third sample.
uint16_t distanceSamples[3] = {0, 0, 0};
uint8_t distanceSampleCount = 0;
uint8_t distanceSampleIndex = 0;
// These counters separate a temporary bad sample from a sensor that is not
// ready or has failed repeatedly.
uint8_t ultrasonicFailures = 0;
uint8_t distanceValidReadings = 0;
uint8_t temperatureTransitionReadings = 0;
uint8_t tiltTransitionReadings = 0;
// The two filters are initialized from the first valid value to avoid a large
// artificial transition from zero to the first real reading.
bool gasFilterInitialized = false;
bool mpuFilterInitialized = false;
// The fallback address supports the two common GY-521 solder-bridge settings.
uint8_t mpuAddress = MPU6050_ADDRESS;
bool mpuReady = false;
// PIR transitions create one event instead of repeating a warning every loop.
bool lastPirState = false;

// Sound sampling state is explicit so the 50 ms window can be progressed in
// small pieces between other control-loop tasks.
bool soundWindowRunning = false;
uint32_t soundWindowStartMs = 0;
uint32_t nextSoundSampleUs = 0;
uint16_t soundMinimum = 1023;
uint16_t soundMaximum = 0;
uint16_t soundSampleCount = 0;

bool elapsed(uint32_t nowMs, uint32_t previousMs, uint32_t periodMs) {
    // Unsigned subtraction remains correct when millis() wraps back to zero.
    return static_cast<uint32_t>(nowMs - previousMs) >= periodMs;
}

// A three-sample median reduces isolated HC-SR04 spikes before safety checks.
// Sorting only the populated prefix also makes the first two readings safe.
uint16_t medianDistance() {
    uint16_t sorted[3] = {distanceSamples[0], distanceSamples[1], distanceSamples[2]};
    const uint8_t count = distanceSampleCount;
    for (uint8_t i = 0; i < count; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < count; ++j) {
            if (sorted[j] < sorted[i]) {
                const uint16_t temporary = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temporary;
            }
        }
    }
    return sorted[count / 2U];
}

// Keep the most recent valid distance and require startup validation before
// the main loop allows forward motion. NewPing returns an echo duration in
// microseconds, and dividing by 58 converts the round-trip time to centimetres.
void readDistance() {
    const uint16_t centimeters = static_cast<uint16_t>(sonar.ping() / 58UL);

    // A zero result means no echo. Values outside the configured sensor range
    // cannot be used for the obstacle decision, so they do not enter the
    // median buffer.
    if (centimeters < ULTRASONIC_MIN_CM || centimeters > ULTRASONIC_MAX_CM) {
        if (ultrasonicFailures < 255U) {
            ++ultrasonicFailures;
        }
        // Before validation, any failed sample restarts the consecutive-valid
        // requirement. After validation, the failure limit controls validity.
        if (ultrasonicFailures >= ULTRASONIC_FAILURE_LIMIT) {
            data.distanceValid = false;
            distanceValidReadings = 0;
        }
        if (distanceValidReadings < ULTRASONIC_VALID_READINGS_REQUIRED) {
            distanceValidReadings = 0;
            data.distanceValid = false;
        }
        return;
    }

    ultrasonicFailures = 0;
    // Store the new sample before calculating the median so a single spike is
    // less likely to change the reported distance used by safety.
    distanceSamples[distanceSampleIndex] = centimeters;
    distanceSampleIndex = static_cast<uint8_t>((distanceSampleIndex + 1U) % 3U);
    if (distanceSampleCount < 3U) {
        ++distanceSampleCount;
    }

    const uint16_t filtered = medianDistance();
    // A newly detected close obstacle is never hidden by an older, larger
    // median value; safety therefore uses the smaller of the two values.
    data.distanceCm = centimeters < filtered ? centimeters : filtered;
    if (distanceValidReadings < 255U) {
        ++distanceValidReadings;
    }
    data.distanceValid =
        distanceValidReadings >= ULTRASONIC_VALID_READINGS_REQUIRED;
}

// The MQ-135 is reported as filtered ADC data rather than calibrated ppm.
void updateGasState() {
    // The heater needs time before comparisons become meaningful. During this
    // period telemetry exposes WARMING instead of raising a false alarm.
    if (!elapsed(millis(), sensorStartMs, GAS_WARMUP_MS)) {
        data.gasState = GAS_WARMING;
        return;
    }

    const uint16_t warning = GAS_BASELINE_ADC + GAS_WARNING_ABOVE_BASELINE;
    const uint16_t critical = GAS_BASELINE_ADC + GAS_CRITICAL_ABOVE_BASELINE;
    // Critical is checked first so a high value cannot be classified as only a
    // warning. Thresholds remain in ADC units because no ppm calibration exists.
    if (data.gasFiltered >= critical) {
        data.gasState = GAS_CRITICAL;
    } else if (data.gasFiltered >= warning) {
        data.gasState = GAS_WARNING;
    } else {
        data.gasState = GAS_NORMAL;
    }
}

void readGas() {
    // Discard the first ADC conversion after switching to this analog channel;
    // the second conversion is less affected by the ADC multiplexer settling.
    (void)analogRead(Pins::MQ135_ANALOG);
    data.gasRaw = static_cast<uint16_t>(analogRead(Pins::MQ135_ANALOG));

    if (!gasFilterInitialized) {
        // Start the filter at the first real sample instead of averaging it with
        // the zero-initialized struct value.
        data.gasFiltered = data.gasRaw;
        gasFilterInitialized = true;
    } else {
        // Move only one quarter of the current error per sample to reduce noise
        // without making a genuine gas increase invisible for too long.
        const int16_t difference = static_cast<int16_t>(data.gasRaw) - static_cast<int16_t>(data.gasFiltered);
        data.gasFiltered = static_cast<uint16_t>(static_cast<int16_t>(data.gasFiltered) + difference / static_cast<int16_t>(GAS_FILTER_DIVISOR));
    }
    updateGasState();
}

// DHT11 readings use deci-units and only confirmed temperature transitions
// change the critical-temperature latch. The DHT library returns false when a
// measurement is invalid, so an invalid sample must not clear a critical state.
void readDht11() {
    data.dhtValid = dht.read();
    if (!data.dhtValid) {
        // Break consecutive confirmation without clearing an active lock.
        temperatureTransitionReadings = 0;
        return;
    }
    // Multiplying by ten preserves one decimal place using integer storage.
    data.temperatureDeciC = static_cast<int16_t>(dht.readTemperature() * 10.0F);
    data.humidityDeciPct = static_cast<uint16_t>(dht.readHumidity() * 10.0F);

    const bool transitionReading = data.temperatureCritical
        ? data.temperatureDeciC < TEMP_CRITICAL_CLEAR_C * 10
        : data.temperatureDeciC >= TEMP_CRITICAL_C * 10;
    // Only readings beyond the appropriate hysteresis boundary advance a
    // transition. Values in the hysteresis band preserve the current state.
    if (!transitionReading) {
        temperatureTransitionReadings = 0;
    } else if (++temperatureTransitionReadings >=
               TEMP_CRITICAL_READINGS_REQUIRED) {
        data.temperatureCritical = !data.temperatureCritical;
        temperatureTransitionReadings = 0;
    }
}

// Convert accelerometer data into filtered pitch and roll angles. The MPU6050
// is used as an accelerometer-only inclinometer; gyro integration is not needed
// for the static tilt safety decision.
void readMpu() {
    if (!mpuReady) {
        // Keep the value invalid so the safety layer does not trust old angles.
        data.mpuValid = false;
        tiltTransitionReadings = 0;
        return;
    }

    Wire.beginTransmission(mpuAddress);
    if (Wire.endTransmission(true) != 0) {
        // A failed I2C probe also resets the filter so a later recovery starts
        // from a fresh sample instead of blending with stale data.
        data.mpuValid = false;
        mpuFilterInitialized = false;
        tiltTransitionReadings = 0;
        return;
    }

    sensors_event_t acceleration;
    if (!mpu.getAccelerometerSensor()->getEvent(&acceleration)) {
        data.mpuValid = false;
        mpuFilterInitialized = false;
        tiltTransitionReadings = 0;
        return;
    }

    constexpr float RAD_TO_DEG_F = 57.2957795F;
    const float ax = acceleration.acceleration.x;
    const float ay = acceleration.acceleration.y;
    const float az = acceleration.acceleration.z;
    if (ax == 0.0F && ay == 0.0F && az == 0.0F) {
        // A zero vector is not a physical 1 g reading and indicates bad data.
        data.mpuValid = false;
        mpuFilterInitialized = false;
        tiltTransitionReadings = 0;
        return;
    }
    const float roll = atan2f(ay, az) * RAD_TO_DEG_F + MPU_ROLL_OFFSET_DEG;
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F + MPU_PITCH_OFFSET_DEG;

    // Integer degrees are sufficient for configured safety thresholds and use
    // much less memory than retaining floating-point angles.
    const int16_t newRoll = static_cast<int16_t>(roll);
    const int16_t newPitch = static_cast<int16_t>(pitch);
    if (!mpuFilterInitialized) {
        // Do not create a false ramp from zero on the first valid reading.
        data.rollDeg = newRoll;
        data.pitchDeg = newPitch;
        mpuFilterInitialized = true;
    } else {
        // A 3:1 old-to-new weighted average damps vibration while following a
        // real tilt change within a few samples.
        data.rollDeg = static_cast<int16_t>((static_cast<int32_t>(data.rollDeg) * 3 + newRoll) / 4);
        data.pitchDeg = static_cast<int16_t>((static_cast<int32_t>(data.pitchDeg) * 3 + newPitch) / 4);
    }
    data.mpuValid = true;

    const int16_t absolutePitch = abs(data.pitchDeg);
    const int16_t absoluteRoll = abs(data.rollDeg);
    const int16_t greatestAngle =
        absolutePitch > absoluteRoll ? absolutePitch : absoluteRoll;
    const bool transitionReading = data.tiltCritical
        ? greatestAngle < TILT_CRITICAL_CLEAR_DEG
        : greatestAngle >= TILT_CRITICAL_DEG;
    if (!transitionReading) {
        tiltTransitionReadings = 0;
    } else if (++tiltTransitionReadings >=
               TILT_CRITICAL_READINGS_REQUIRED) {
        data.tiltCritical = !data.tiltCritical;
        tiltTransitionReadings = 0;
    }
}

// The light module is read through AO; the configured polarity determines
// whether the raw value represents darkness. Keeping the raw value in telemetry
// makes later threshold calibration possible without changing the wiring.
void readLight() {
    data.lightRaw = static_cast<uint16_t>(analogRead(Pins::LIGHT_ANALOG));
    data.dark = LIGHT_ANALOG_DARK_BELOW
                    ? data.lightRaw < LIGHT_DARK_THRESHOLD
                    : data.lightRaw >= LIGHT_DARK_THRESHOLD;
}

// PIR events are accepted only after startup and while the rover is stationary.
// Motor vibration can otherwise create false motion reports.
void readPir(bool motorsMoving) {
    const bool ready = elapsed(millis(), sensorStartMs, PIR_STARTUP_MS);
    const bool current = ready && !motorsMoving && digitalRead(Pins::PIR) == HIGH;
    data.pirMotion = current;
    // Rising-edge detection prevents the same moving person from generating a
    // new Bluetooth message on every loop iteration.
    if (current && !lastPirState) {
        pendingEvents |= SENSOR_EVENT_MOTION;
    }
    lastPirState = current;
}

// Sample sound incrementally so the 50 ms window does not block the loop. Four
// samples per pass bounds the work even if the main loop is briefly delayed.
void updateSoundWindow(uint32_t nowMs) {
    if (!soundWindowRunning) {
        return;
    }

    uint8_t samplesThisPass = 0;
    uint32_t nowUs = micros();
    while (static_cast<int32_t>(nowUs - nextSoundSampleUs) >= 0 && samplesThisPass < 4U) {
        // Min/max captures amplitude without allocating a waveform buffer.
        const uint16_t sample = static_cast<uint16_t>(analogRead(Pins::SOUND_ANALOG));
        if (sample < soundMinimum) {
            soundMinimum = sample;
        }
        if (sample > soundMaximum) {
            soundMaximum = sample;
        }
        ++soundSampleCount;
        ++samplesThisPass;
        nextSoundSampleUs += SOUND_SAMPLE_INTERVAL_US;
        nowUs = micros();
    }

    if (!elapsed(nowMs, soundWindowStartMs, SOUND_SAMPLE_WINDOW_MS)) {
        return;
    }

    soundWindowRunning = false;
    // One sample cannot define an amplitude, so it is treated as silence.
    data.soundAmplitude = soundSampleCount > 1U ? soundMaximum - soundMinimum : 0U;
    if (data.soundAmplitude >= SOUND_THRESHOLD) {
        pendingEvents |= SENSOR_EVENT_SOUND;
    }
}

}  // namespace

void sensorsBegin() {
    // Configure pins and initialise all external sensor libraries once. Keeping
    // setup here means the main sketch does not need sensor-specific ordering.
    pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
    pinMode(Pins::ULTRASONIC_ECHO, INPUT);
    pinMode(Pins::MQ135_ANALOG, INPUT);
    pinMode(Pins::SOUND_ANALOG, INPUT);
    pinMode(Pins::LIGHT_ANALOG, INPUT);
    pinMode(Pins::PIR, INPUT);
    digitalWrite(Pins::ULTRASONIC_TRIG, LOW);

    dht.begin();
    Wire.begin();
    // A timeout prevents a disconnected I2C device from holding the whole loop.
    Wire.setWireTimeout(3000UL, true);
    mpuReady = mpu.begin(mpuAddress, &Wire);
    if (!mpuReady) {
        // Retry the alternate address used by boards with the AD0 bridge high.
        mpuAddress = MPU6050_ADDRESS == 0x68U ? 0x69U : 0x68U;
        mpuReady = mpu.begin(mpuAddress, &Wire);
    }
    if (mpuReady) {
        // A small range and moderate bandwidth give stable tilt values for a
        // slow rover without amplifying high-frequency vibration.
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    }
    data.mpuValid = mpuReady;
    data.gasState = GAS_WARMING;
    distanceValidReadings = 0;
    data.distanceValid = false;

    sensorStartMs = millis();
    // Make the first scheduled pass happen immediately after setup instead of
    // waiting for a full period while telemetry reports zero values.
    lastMpuMs = sensorStartMs - MPU_PERIOD_MS;
    lastDistanceMs = sensorStartMs - ULTRASONIC_PERIOD_MS;
    lastGasMs = sensorStartMs - GAS_PERIOD_MS;
    lastLightMs = sensorStartMs - LIGHT_PERIOD_MS;
    lastDhtMs = sensorStartMs - DHT_PERIOD_MS;
    lastSoundWindowMs = sensorStartMs - SOUND_IDLE_PERIOD_MS;
}

void sensorsUpdate(uint32_t nowMs, bool motorsMoving, bool scanMode) {
    // Schedule independent reads so slow sensors do not run on every loop. The
    // loop remains responsive to Bluetooth while each sensor keeps its period.
    if (elapsed(nowMs, lastMpuMs, MPU_PERIOD_MS)) {
        lastMpuMs = nowMs;
        readMpu();
    }
    if (elapsed(nowMs, lastDistanceMs, ULTRASONIC_PERIOD_MS)) {
        lastDistanceMs = nowMs;
        readDistance();
    }
    if (elapsed(nowMs, lastGasMs, GAS_PERIOD_MS)) {
        lastGasMs = nowMs;
        readGas();
    }
    if (elapsed(nowMs, lastLightMs, LIGHT_PERIOD_MS)) {
        lastLightMs = nowMs;
        readLight();
    }
    if (elapsed(nowMs, lastDhtMs, DHT_PERIOD_MS)) {
        lastDhtMs = nowMs;
        readDht11();
    }

    readPir(motorsMoving);
    updateSoundWindow(nowMs);
    // Automatic sound windows run only while stationary and outside scan mode;
    // scan mode starts its own deliberate window.
    if (!motorsMoving && !scanMode && !soundWindowRunning && elapsed(nowMs, lastSoundWindowMs, SOUND_IDLE_PERIOD_MS)) {
        sensorsStartSoundWindow(nowMs);
    }
}

const SensorData& sensorsGetData() {
    return data;
}

void sensorsStartSoundWindow(uint32_t nowMs) {
    if (soundWindowRunning) {
        // Do not reset an active window when two callers request it together.
        return;
    }

    // Prime the ADC channel before collecting min/max samples.
    (void)analogRead(Pins::SOUND_ANALOG);
    soundMinimum = 1023;
    soundMaximum = 0;
    soundSampleCount = 0;
    soundWindowStartMs = nowMs;
    lastSoundWindowMs = nowMs;
    nextSoundSampleUs = micros();
    soundWindowRunning = true;
}

bool sensorsSoundWindowActive() {
    return soundWindowRunning;
}

void sensorsRefreshForScan(uint32_t nowMs) {
    // A scan refreshes the values that are useful in a single operator report.
    // It does not duplicate the entire normal scheduler or create a second
    // SensorData object.
    readPir(false);
    if (elapsed(nowMs, lastDhtMs, DHT_PERIOD_MS)) {
        lastDhtMs = nowMs;
        readDht11();
    }
    readGas();
    lastGasMs = nowMs;
    readDistance();
    lastDistanceMs = nowMs;
    readMpu();
    lastMpuMs = nowMs;
    readLight();
    lastLightMs = nowMs;
}

uint8_t sensorsConsumeEvents() {
    // Copy then clear makes each event a one-shot notification to main.cpp.
    const uint8_t events = pendingEvents;
    pendingEvents = 0;
    return events;
}
