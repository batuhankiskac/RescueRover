#include "sensors.hpp"

#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <NewPing.h>
#include <Wire.h>
#include <math.h>
#include "config.hpp"
#include "pins.hpp"

namespace {

DHT dht(Pins::DHT11_DATA, DHT11);
Adafruit_MPU6050 mpu;
NewPing sonar(Pins::ULTRASONIC_TRIG, Pins::ULTRASONIC_ECHO,
              ULTRASONIC_MAX_CM);

SensorData data = {};
uint8_t pendingEvents = 0;
uint32_t sensorStartMs = 0;

uint32_t lastMpuMs = 0;
uint32_t lastDistanceMs = 0;
uint32_t lastGasMs = 0;
uint32_t lastLightMs = 0;
uint32_t lastDhtMs = 0;
uint32_t lastSoundWindowMs = 0;

uint16_t distanceSamples[3] = {0, 0, 0};
uint8_t distanceSampleCount = 0;
uint8_t distanceSampleIndex = 0;
uint8_t ultrasonicFailures = 0;
uint8_t distanceValidReadings = 0;
uint8_t temperatureTransitionReadings = 0;
bool gasFilterInitialized = false;
bool mpuFilterInitialized = false;
uint8_t mpuAddress = MPU6050_ADDRESS;
bool mpuReady = false;
bool lastPirState = false;

bool soundWindowRunning = false;
uint32_t soundWindowStartMs = 0;
uint32_t nextSoundSampleUs = 0;
uint16_t soundMinimum = 1023;
uint16_t soundMaximum = 0;
uint16_t soundSampleCount = 0;

bool elapsed(uint32_t nowMs, uint32_t previousMs, uint32_t periodMs) {
    return static_cast<uint32_t>(nowMs - previousMs) >= periodMs;
}

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

void readDistance() {
    const uint16_t centimeters = static_cast<uint16_t>(sonar.ping() / 58UL);

    if (centimeters < ULTRASONIC_MIN_CM || centimeters > ULTRASONIC_MAX_CM) {
        if (ultrasonicFailures < 255U) {
            ++ultrasonicFailures;
        }
        if (ultrasonicFailures >= ULTRASONIC_FAILURE_LIMIT) {
            data.distanceValid = false;
        }
        if (distanceValidReadings < ULTRASONIC_VALID_READINGS_REQUIRED) {
            distanceValidReadings = 0;
            data.distanceValid = false;
        }
        return;
    }

    ultrasonicFailures = 0;
    distanceSamples[distanceSampleIndex] = centimeters;
    distanceSampleIndex = static_cast<uint8_t>((distanceSampleIndex + 1U) % 3U);
    if (distanceSampleCount < 3U) {
        ++distanceSampleCount;
    }

    const uint16_t filtered = medianDistance();
    data.distanceCm = centimeters < filtered ? centimeters : filtered;
    if (distanceValidReadings < 255U) {
        ++distanceValidReadings;
    }
    data.distanceValid =
        distanceValidReadings >= ULTRASONIC_VALID_READINGS_REQUIRED;
}

void updateGasState() {
    if (!elapsed(millis(), sensorStartMs, GAS_WARMUP_MS)) {
        data.gasState = GAS_WARMING;
        return;
    }

    const uint16_t warning = GAS_BASELINE_ADC + GAS_WARNING_ABOVE_BASELINE;
    const uint16_t critical = GAS_BASELINE_ADC + GAS_CRITICAL_ABOVE_BASELINE;
    if (data.gasFiltered >= critical) {
        data.gasState = GAS_CRITICAL;
    } else if (data.gasFiltered >= warning) {
        data.gasState = GAS_WARNING;
    } else {
        data.gasState = GAS_NORMAL;
    }
}

void readGas() {
    (void)analogRead(Pins::MQ135_ANALOG);
    data.gasRaw = static_cast<uint16_t>(analogRead(Pins::MQ135_ANALOG));

    if (!gasFilterInitialized) {
        data.gasFiltered = data.gasRaw;
        gasFilterInitialized = true;
    } else {
        const int16_t difference = static_cast<int16_t>(data.gasRaw) - static_cast<int16_t>(data.gasFiltered);
        data.gasFiltered = static_cast<uint16_t>(static_cast<int16_t>(data.gasFiltered) + difference / static_cast<int16_t>(GAS_FILTER_DIVISOR));
    }
    updateGasState();
}

void readDht11() {
    data.dhtValid = dht.read();
    if (!data.dhtValid) {
        // Break consecutive confirmation without clearing an active lock.
        temperatureTransitionReadings = 0;
        return;
    }
    data.temperatureDeciC = static_cast<int16_t>(dht.readTemperature() * 10.0F);
    data.humidityDeciPct = static_cast<uint16_t>(dht.readHumidity() * 10.0F);

    const bool criticalReading =
        data.temperatureDeciC >= TEMP_CRITICAL_C * 10;
    if (criticalReading == data.temperatureCritical) {
        temperatureTransitionReadings = 0;
    } else if (++temperatureTransitionReadings >=
               TEMP_CRITICAL_READINGS_REQUIRED) {
        data.temperatureCritical = criticalReading;
        temperatureTransitionReadings = 0;
    }
}

void readMpu() {
    if (!mpuReady) {
        data.mpuValid = false;
        return;
    }

    Wire.beginTransmission(mpuAddress);
    if (Wire.endTransmission(true) != 0) {
        data.mpuValid = false;
        mpuFilterInitialized = false;
        return;
    }

    sensors_event_t acceleration;
    if (!mpu.getAccelerometerSensor()->getEvent(&acceleration)) {
        data.mpuValid = false;
        mpuFilterInitialized = false;
        return;
    }

    constexpr float RAD_TO_DEG_F = 57.2957795F;
    const float ax = acceleration.acceleration.x;
    const float ay = acceleration.acceleration.y;
    const float az = acceleration.acceleration.z;
    if (ax == 0.0F && ay == 0.0F && az == 0.0F) {
        data.mpuValid = false;
        mpuFilterInitialized = false;
        return;
    }
    const float roll = atan2f(ay, az) * RAD_TO_DEG_F + MPU_ROLL_OFFSET_DEG;
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F + MPU_PITCH_OFFSET_DEG;

    const int16_t newRoll = static_cast<int16_t>(roll);
    const int16_t newPitch = static_cast<int16_t>(pitch);
    if (!mpuFilterInitialized) {
        data.rollDeg = newRoll;
        data.pitchDeg = newPitch;
        mpuFilterInitialized = true;
    } else {
        data.rollDeg = static_cast<int16_t>((static_cast<int32_t>(data.rollDeg) * 3 + newRoll) / 4);
        data.pitchDeg = static_cast<int16_t>((static_cast<int32_t>(data.pitchDeg) * 3 + newPitch) / 4);
    }
    data.mpuValid = true;
}

void readLight() {
    data.lightRaw = static_cast<uint16_t>(analogRead(Pins::LIGHT_ANALOG));
    data.dark = LIGHT_ANALOG_DARK_BELOW
                    ? data.lightRaw < LIGHT_DARK_THRESHOLD
                    : data.lightRaw >= LIGHT_DARK_THRESHOLD;
}

void readPir(bool motorsMoving) {
    const bool ready = elapsed(millis(), sensorStartMs, PIR_STARTUP_MS);
    const bool current = ready && !motorsMoving && digitalRead(Pins::PIR) == HIGH;
    data.pirMotion = current;
    if (current && !lastPirState) {
        pendingEvents |= SENSOR_EVENT_MOTION;
    }
    lastPirState = current;
}

void updateSoundWindow(uint32_t nowMs) {
    if (!soundWindowRunning) {
        return;
    }

    uint8_t samplesThisPass = 0;
    uint32_t nowUs = micros();
    while (static_cast<int32_t>(nowUs - nextSoundSampleUs) >= 0 && samplesThisPass < 4U) {
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
    data.soundAmplitude = soundSampleCount > 1U ? soundMaximum - soundMinimum : 0U;
    if (data.soundAmplitude >= SOUND_THRESHOLD) {
        pendingEvents |= SENSOR_EVENT_SOUND;
    }
}

}  // namespace

void sensorsBegin() {
    pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
    pinMode(Pins::ULTRASONIC_ECHO, INPUT);
    pinMode(Pins::MQ135_ANALOG, INPUT);
    pinMode(Pins::SOUND_ANALOG, INPUT);
    pinMode(Pins::LIGHT_ANALOG, INPUT);
    pinMode(Pins::PIR, INPUT);
    digitalWrite(Pins::ULTRASONIC_TRIG, LOW);

    dht.begin();
    Wire.begin();
    Wire.setWireTimeout(3000UL, true);
    mpuReady = mpu.begin(mpuAddress, &Wire);
    if (!mpuReady) {
        mpuAddress = MPU6050_ADDRESS == 0x68U ? 0x69U : 0x68U;
        mpuReady = mpu.begin(mpuAddress, &Wire);
    }
    if (mpuReady) {
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    }
    data.mpuValid = mpuReady;
    data.gasState = GAS_WARMING;
    distanceValidReadings = 0;
    data.distanceValid = false;

    sensorStartMs = millis();
    lastMpuMs = sensorStartMs - MPU_PERIOD_MS;
    lastDistanceMs = sensorStartMs - ULTRASONIC_PERIOD_MS;
    lastGasMs = sensorStartMs - GAS_PERIOD_MS;
    lastLightMs = sensorStartMs - LIGHT_PERIOD_MS;
    lastDhtMs = sensorStartMs - DHT_PERIOD_MS;
    lastSoundWindowMs = sensorStartMs - SOUND_IDLE_PERIOD_MS;
}

void sensorsUpdate(uint32_t nowMs, bool motorsMoving, bool scanMode) {
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
    if (!motorsMoving && !scanMode && !soundWindowRunning && elapsed(nowMs, lastSoundWindowMs, SOUND_IDLE_PERIOD_MS)) {
        sensorsStartSoundWindow(nowMs);
    }
}

const SensorData& sensorsGetData() {
    return data;
}

void sensorsStartSoundWindow(uint32_t nowMs) {
    if (soundWindowRunning) {
        return;
    }

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
    const uint8_t events = pendingEvents;
    pendingEvents = 0;
    return events;
}
