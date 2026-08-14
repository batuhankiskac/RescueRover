#pragma once

#include <Arduino.h>
#include "config.h"

#if DEBUG_ENABLED
#define DEBUG_BEGIN(baud) Serial.begin(baud)
#define DEBUG_PRINT(value) Serial.print(value)
#define DEBUG_PRINTLN(value) Serial.println(value)
#else
#define DEBUG_BEGIN(baud) do { (void)(baud); } while (0)
#define DEBUG_PRINT(value) do { } while (0)
#define DEBUG_PRINTLN(value) do { } while (0)
#endif
