#pragma once

#include <Arduino.h>

class SoftwareSerial : public Print {
public:
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin);

    void begin(unsigned long baud);
    bool listen();
    int available();
    int read();
    size_t write(uint8_t value) override;
};
