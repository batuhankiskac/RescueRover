#include "bluetooth.hpp"

#include <SoftwareSerial.h>

#include "config.hpp"
#include "pins.hpp"

namespace {

    SoftwareSerial bluetoothSerial(Pins::BT_RX, Pins::BT_TX);

    BluetoothCommand decodeCommand(char value) {
        if (value >= 'a' && value <= 'z') {
            value = static_cast < char >(value - ('a' - 'A'));
        }

        switch (value) {
            case 'F':
            case 'W':
            return BluetoothCommand::FORWARD;
            case 'B':
            return BluetoothCommand::BACKWARD;
            case 'L':
            case 'A':
            return BluetoothCommand::LEFT;
            case 'R':
            case 'D':
            return BluetoothCommand::RIGHT;
            case 'S':
            return BluetoothCommand::STOP;
            case 'T':
            return BluetoothCommand::SCAN;
            case 'X':
            return BluetoothCommand::EMERGENCY_STOP;
            case 'C':
            return BluetoothCommand::CLEAR_EMERGENCY;
            case 'P':
            return BluetoothCommand::HEARTBEAT;
            case 'H':
            return BluetoothCommand::LIGHT_ON;
            case 'J':
            return BluetoothCommand::LIGHT_OFF;
            case 'U':
            return BluetoothCommand::LIGHT_AUTO;
            case '?':
            return BluetoothCommand::HELP;
            case '0':return BluetoothCommand::SPEED_0;
            case '1':return BluetoothCommand::SPEED_1;
            case '2':return BluetoothCommand::SPEED_2;
            case '3':return BluetoothCommand::SPEED_3;
            case '4':return BluetoothCommand::SPEED_4;
            case '5':return BluetoothCommand::SPEED_5;
            case '6':return BluetoothCommand::SPEED_6;
            case '7':return BluetoothCommand::SPEED_7;
            case '8':return BluetoothCommand::SPEED_8;
            case '9':return BluetoothCommand::SPEED_9;
            default:
            return BluetoothCommand::NONE;
        }
    }

}

void bluetoothBegin() {
    bluetoothSerial.begin(BT_BAUD_RATE);
    bluetoothSerial.listen();
}

BluetoothCommand bluetoothPoll() {
    if (bluetoothSerial.available() <= 0) {
return BluetoothCommand::NONE;
    }

    return decodeCommand(static_cast < char >(bluetoothSerial.read()));
}

Print & bluetoothOutput() {
    return bluetoothSerial;
}

void bluetoothSendHelp() {
    bluetoothSerial.println(
                            F("CMD:F/W,B,L/A,R/D,S,T,X,C,P,H,J,U,0-9,?"));
}
