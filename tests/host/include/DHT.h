#pragma once

#include <Arduino.h>

constexpr uint8_t DHT11 = 11;

class DHT {
public:
    DHT(uint8_t pin, uint8_t type);

    void begin();
    bool read();
    float readTemperature();
    float readHumidity();
};
