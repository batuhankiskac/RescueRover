#include "host_fake.hpp"

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include <array>
#include <deque>
#include <sstream>

namespace {

struct State {
    uint32_t nowMs = 0;
    std::array<int, 32> digital = {};
    std::array<int, 32> analog = {};
    std::array<uint16_t, 32> analogDefault = {};
    std::array<std::deque<uint16_t>, 32> analogScript;
    std::deque<uint16_t> pingCm;
    uint16_t lastPingCm = 50;
    std::deque<host::DhtSample> dht;
    host::DhtSample currentDht = {true, 20.0F, 50.0F};
    std::deque<host::Acceleration> mpu;
    host::Acceleration mpuDefault = {0.0F, 0.0F, 1.0F};
    bool mpuBeginResult = true;
    uint8_t wireResult = 0;
    std::deque<char> bluetoothRx;
    std::string activeLine;
    std::vector<std::string> serial;
    std::vector<host::Write> writeLog;
};

State &state() {
    static State instance;
    return instance;
}

size_t append(Print &output, const char *text) {
    size_t count = 0;
    while (*text != '\0') {
        count += output.write(static_cast<uint8_t>(*text++));
    }
    return count;
}

template <typename Number>
size_t appendNumber(Print &output, Number value) {
    std::ostringstream stream;
    stream << value;
    return append(output, stream.str().c_str());
}

}  // namespace

namespace host {

void resetIo() {
    state() = State();
}

void setTime(uint32_t milliseconds) {
    state().nowMs = milliseconds;
}

void setDigitalInput(uint8_t pin, bool high) {
    state().digital[pin] = high ? HIGH : LOW;
}

void setAnalogDefault(uint8_t pin, uint16_t value) {
    state().analogDefault[pin] = value;
}

void setAnalogScript(uint8_t pin, const std::vector<uint16_t> &values) {
    state().analogScript[pin] = std::deque<uint16_t>(values.begin(), values.end());
}

void setPingCmScript(const std::vector<uint16_t> &values) {
    state().pingCm = std::deque<uint16_t>(values.begin(), values.end());
}

void setDhtScript(const std::vector<DhtSample> &values) {
    state().dht = std::deque<DhtSample>(values.begin(), values.end());
}

void setMpuDefault(Acceleration value) {
    state().mpuDefault = value;
}

void setMpuScript(const std::vector<Acceleration> &values) {
    state().mpu = std::deque<Acceleration>(values.begin(), values.end());
}

void setMpuBeginResult(bool value) {
    state().mpuBeginResult = value;
}

void setWireResult(uint8_t value) {
    state().wireResult = value;
}

void pushBluetooth(const std::string &commands) {
    for (char command : commands) {
        state().bluetoothRx.push_back(command);
    }
}

void clearSerial() {
    state().serial.clear();
    state().activeLine.clear();
}

void clearWrites() {
    state().writeLog.clear();
}

const std::vector<std::string> &serialLines() {
    return state().serial;
}

const std::vector<Write> &writes() {
    return state().writeLog;
}

int digitalValue(uint8_t pin) {
    return state().digital[pin];
}

int analogValue(uint8_t pin) {
    return state().analog[pin];
}

}  // namespace host

size_t Print::print(const char *text) {
    return append(*this, text);
}

size_t Print::print(const __FlashStringHelper *text) {
    return append(*this, reinterpret_cast<const char *>(text));
}

size_t Print::print(char value) {
    return write(static_cast<uint8_t>(value));
}

size_t Print::print(uint8_t value) {
    return appendNumber(*this, static_cast<unsigned int>(value));
}

size_t Print::print(uint16_t value) {
    return appendNumber(*this, value);
}

size_t Print::print(uint32_t value) {
    return appendNumber(*this, value);
}

size_t Print::print(int16_t value) {
    return appendNumber(*this, value);
}

size_t Print::print(int value) {
    return appendNumber(*this, value);
}

size_t Print::println() {
    return write('\n');
}

size_t Print::println(const char *text) {
    return print(text) + println();
}

size_t Print::println(const __FlashStringHelper *text) {
    return print(text) + println();
}

uint32_t millis() {
    return state().nowMs;
}

uint32_t micros() {
    return state().nowMs * 1000UL;
}

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t value) {
    state().digital[pin] = value;
    state().writeLog.push_back({host::Write::DIGITAL, pin, value, state().nowMs});
}

int digitalRead(uint8_t pin) {
    return state().digital[pin];
}

int analogRead(uint8_t pin) {
    std::deque<uint16_t> &script = state().analogScript[pin];
    if (!script.empty()) {
        const uint16_t value = script.front();
        script.pop_front();
        return value;
    }
    return state().analogDefault[pin];
}

void analogWrite(uint8_t pin, int value) {
    state().analog[pin] = value;
    state().writeLog.push_back({host::Write::ANALOG, pin, value, state().nowMs});
}

SoftwareSerial::SoftwareSerial(uint8_t, uint8_t) {}

void SoftwareSerial::begin(unsigned long) {}

bool SoftwareSerial::listen() {
    return true;
}

int SoftwareSerial::available() {
    return static_cast<int>(state().bluetoothRx.size());
}

int SoftwareSerial::read() {
    if (state().bluetoothRx.empty()) {
        return -1;
    }
    const char value = state().bluetoothRx.front();
    state().bluetoothRx.pop_front();
    return static_cast<unsigned char>(value);
}

size_t SoftwareSerial::write(uint8_t value) {
    if (value == '\n') {
        state().serial.push_back(state().activeLine);
        state().activeLine.clear();
    } else {
        state().activeLine.push_back(static_cast<char>(value));
    }
    return 1;
}

DHT::DHT(uint8_t, uint8_t) {}

void DHT::begin() {}

bool DHT::read() {
    if (!state().dht.empty()) {
        state().currentDht = state().dht.front();
        state().dht.pop_front();
    }
    return state().currentDht.valid;
}

float DHT::readTemperature() {
    return state().currentDht.temperatureC;
}

float DHT::readHumidity() {
    return state().currentDht.humidityPct;
}

NewPing::NewPing(uint8_t, uint8_t, uint16_t) {}

unsigned long NewPing::ping() {
    if (!state().pingCm.empty()) {
        state().lastPingCm = state().pingCm.front();
        state().pingCm.pop_front();
    }
    return static_cast<unsigned long>(state().lastPingCm) * 58UL;
}

TwoWire Wire;

void TwoWire::begin() {}

void TwoWire::setWireTimeout(uint32_t, bool) {}

void TwoWire::beginTransmission(uint8_t) {}

uint8_t TwoWire::endTransmission(bool) {
    return state().wireResult;
}

bool Adafruit_MPU6050::begin(uint8_t, TwoWire *) {
    return state().mpuBeginResult;
}

void Adafruit_MPU6050::setAccelerometerRange(mpu6050_accel_range_t) {}

void Adafruit_MPU6050::setFilterBandwidth(mpu6050_bandwidth_t) {}

Adafruit_MPU6050::Accelerometer *Adafruit_MPU6050::getAccelerometerSensor() {
    return &accelerometer_;
}

bool Adafruit_MPU6050::Accelerometer::getEvent(sensors_event_t *event) {
    host::Acceleration value = state().mpuDefault;
    if (!state().mpu.empty()) {
        value = state().mpu.front();
        state().mpu.pop_front();
    }
    event->acceleration.x = value.x;
    event->acceleration.y = value.y;
    event->acceleration.z = value.z;
    return true;
}
