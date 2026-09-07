#pragma once

#include <Arduino.h>

class NewPing {
public:
    NewPing(uint8_t triggerPin, uint8_t echoPin, uint16_t maxDistanceCm);

    unsigned long ping();
};
