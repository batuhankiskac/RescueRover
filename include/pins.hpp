#pragma once

#include <Arduino.h>
namespace Pins {
constexpr uint8_t MQ135_ANALOG = A0;
constexpr uint8_t SOUND_ANALOG = A1;
constexpr uint8_t LIGHT_ANALOG = A2;

constexpr uint8_t HEADLIGHT = 13;
constexpr uint8_t ALARM = A3;

constexpr uint8_t PIR = 2;

constexpr uint8_t MOTOR_ENABLE_PWM = 3;
constexpr uint8_t MOTOR_LEFT_IN1 = 4;
constexpr uint8_t MOTOR_LEFT_IN2 = 5;
constexpr uint8_t MOTOR_RIGHT_IN1 = 6;
constexpr uint8_t MOTOR_RIGHT_IN2 = 7;

constexpr uint8_t ULTRASONIC_TRIG = 8;
constexpr uint8_t ULTRASONIC_ECHO = 9;

constexpr uint8_t ESP32_LINK_RX = 10;
constexpr uint8_t ESP32_LINK_TX = 11;

constexpr uint8_t DHT11_DATA = 12;
}
