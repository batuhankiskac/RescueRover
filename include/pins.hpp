#pragma once

#include <Arduino.h>
namespace Pins {
// A0 is dedicated to the MQ-135 so its raw ADC value can be filtered in the
// sensor module and compared with the configured gas baseline.
constexpr uint8_t MQ135_ANALOG = A0;
// A1 reads the microphone module's analog output. The digital output is not
// used because the firmware needs an amplitude value for its own threshold.
constexpr uint8_t SOUND_ANALOG = A1;
// A2 is used for the light module's AO pin; the module's DO pin is unused.
constexpr uint8_t LIGHT_ANALOG = A2;

// D13 drives the headlight transistor or MOSFET, so the Uno pin does not carry
// the LED current directly. The output is controlled automatically from AO.
constexpr uint8_t HEADLIGHT = 13;
// A3 drives the active buzzer for short critical-event beeps.
constexpr uint8_t ALARM = A3;

// The PIR module reports motion through its digital OUT pin.
constexpr uint8_t PIR = 2;

// D3 is shared by ENA and ENB on the L298N, which keeps both motor sides at
// the same PWM speed while saving an Uno output pin.
constexpr uint8_t MOTOR_ENABLE_PWM = 3;
// These four pins select the polarity of the left and right motor sides.
constexpr uint8_t MOTOR_LEFT_IN1 = 4;
constexpr uint8_t MOTOR_LEFT_IN2 = 5;
constexpr uint8_t MOTOR_RIGHT_IN1 = 6;
constexpr uint8_t MOTOR_RIGHT_IN2 = 7;

// The trigger is driven by the Uno and the echo pulse is measured by NewPing.
constexpr uint8_t ULTRASONIC_TRIG = 8;
constexpr uint8_t ULTRASONIC_ECHO = 9;

// SoftwareSerial receives HC-05 TXD on D10 and sends on D11. D11 must use a
// voltage divider before it reaches the HC-05 RXD input.
constexpr uint8_t BT_RX = 10;
constexpr uint8_t BT_TX = 11;
// The HC-05 STATE signal is read on D0 to reject motion when Bluetooth drops.
constexpr uint8_t BT_STATE = 0;

// D12 is shared bidirectionally with the DHT11 data line.
constexpr uint8_t DHT11_DATA = 12;
}
