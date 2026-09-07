#pragma once

#include <stdint.h>

#include <string>
#include <vector>

namespace host {

struct DhtSample {
    bool valid;
    float temperatureC;
    float humidityPct;
};

struct Acceleration {
    float x;
    float y;
    float z;
};

struct Write {
    enum Kind { DIGITAL, ANALOG } kind;
    uint8_t pin;
    int value;
    uint32_t atMs;
};

void resetIo();
void setTime(uint32_t milliseconds);
void setDigitalInput(uint8_t pin, bool high);
void setAnalogDefault(uint8_t pin, uint16_t value);
void setAnalogScript(uint8_t pin, const std::vector<uint16_t> &values);
void setPingCmScript(const std::vector<uint16_t> &values);
void setDhtScript(const std::vector<DhtSample> &values);
void setMpuDefault(Acceleration value);
void setMpuScript(const std::vector<Acceleration> &values);
void setMpuBeginResult(bool value);
void setWireResult(uint8_t value);
void pushBluetooth(const std::string &commands);
void clearSerial();
void clearWrites();

const std::vector<std::string> &serialLines();
const std::vector<Write> &writes();
int digitalValue(uint8_t pin);
int analogValue(uint8_t pin);

}  // namespace host
