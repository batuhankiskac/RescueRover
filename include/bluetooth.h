#pragma once

#include <Arduino.h>
#include "rover_types.h"

void			bluetoothBegin();
BluetoothCommand bluetoothPoll();
Print & bluetoothOutput();
void			bluetoothSendHelp();
