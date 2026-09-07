#pragma once

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>

using byte = uint8_t;

class __FlashStringHelper;

#define F(text) (reinterpret_cast<const __FlashStringHelper *>(text))

constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t INPUT = 0;
constexpr uint8_t OUTPUT = 1;
constexpr uint8_t A0 = 14;
constexpr uint8_t A1 = 15;
constexpr uint8_t A2 = 16;
constexpr uint8_t A3 = 17;

class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t value) = 0;

    size_t print(const char *text);
    size_t print(const __FlashStringHelper *text);
    size_t print(char value);
    size_t print(uint8_t value);
    size_t print(uint16_t value);
    size_t print(uint32_t value);
    size_t print(int16_t value);
    size_t print(int value);

    size_t println();
    size_t println(const char *text);
    size_t println(const __FlashStringHelper *text);
};

uint32_t millis();
uint32_t micros();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
