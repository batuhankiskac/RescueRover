#pragma once

#include <Arduino.h>
#include "rover_types.hpp"

void bluetoothBegin();
BluetoothCommand bluetoothPoll();
Print& bluetoothOutput();
void bluetoothSendHelp();
