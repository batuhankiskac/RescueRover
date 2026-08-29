#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic support is disabled for this ESP32 target"
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth SPP is unavailable; use an original ESP32, not ESP32-C3/S3"
#endif

namespace {
constexpr char BLUETOOTH_DEVICE_NAME[] = "RescueRover";
constexpr uint32_t ROVER_UART_BAUD_RATE = 9600UL;
constexpr int8_t ROVER_UART_RX = 16;
constexpr int8_t ROVER_UART_TX = 17;

BluetoothSerial bluetooth;
HardwareSerial roverUart(2);

void forwardAvailable(Stream& input, Stream& output) {
    while (input.available() > 0) {
        output.write((uint8_t)input.read());
    }
}
}

void setup() {
    roverUart.begin(ROVER_UART_BAUD_RATE, SERIAL_8N1,
                    ROVER_UART_RX, ROVER_UART_TX);
    bluetooth.begin(BLUETOOTH_DEVICE_NAME);
}

void loop() {
    forwardAvailable(bluetooth, roverUart);
    forwardAvailable(roverUart, bluetooth);
}
