#pragma once

#include <Arduino.h>

class TwoWire {
public:
    void begin();
    void setWireTimeout(uint32_t timeout, bool resetWithTimeout);
    void beginTransmission(uint8_t address);
    uint8_t endTransmission(bool sendStop);
};

extern TwoWire Wire;
