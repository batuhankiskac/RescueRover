#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

void motorBegin();

bool moveForward(uint8_t speed);
bool moveBackward(uint8_t speed);
bool turnLeft(uint8_t speed);
bool turnRight(uint8_t speed);
void stopMotors();

void motorSetSafetyInhibit(bool blockAllMotion, bool blockForward);
const MotorStatus& motorGetStatus();
bool motorIsMoving();
