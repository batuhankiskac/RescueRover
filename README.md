# Rescue Rover firmware

PlatformIO firmware for an Arduino Uno R3 search-and-rescue rover. It supports
Bluetooth driving, scheduled sensing, automatic headlights, scan mode, compact
telemetry, and safety-enforced motor stopping.

This is prototype equipment, not a certified gas, fire, structural-safety, or
life-detection instrument. Never send it into a hazardous environment until it
has been electrically checked and tested in a controlled area.

## Final pin mapping

| Uno pin | Connection | Direction / note |
|---|---|---|
| D0 / D1 | USB / upload | Not used by the firmware; kept free for uploading |
| D2 | HC-SR501 OUT | Digital input |
| D3 | L298N ENA **and** ENB | Shared PWM output; remove both EN jumpers |
| D4 | L298N IN1 | Left motor direction |
| D5 | L298N IN2 | Left motor direction |
| D6 | L298N IN3 | Right motor direction |
| D7 | L298N IN4 | Right motor direction |
| D8 | HC-SR04 TRIG | Digital output |
| D9 | HC-SR04 ECHO | Digital input |
| D10 | HC-05 TXD | SoftwareSerial RX; direct connection is normally safe |
| D11 | HC-05 RXD | SoftwareSerial TX **through a 5 V-to-3.3 V divider** |
| D12 | DHT11 OUT | Digital bidirectional data |
| D13 | Headlight driver | Digital output |
| A0 | MQ-135 AO | Analog input |
| A1 | Sound module AO | Analog input |
| A2 | 4-pin light module AO | Analog input |
| A3 | Active buzzer | Digital output; shared alarm output |
| A4 | MPU6050 SDA | I2C |
| A5 | MPU6050 SCL | I2C |

There are no pin conflicts. SoftwareSerial uses D10/D11, Timer 2 PWM is used on
D3, and the MPU6050 owns A4/A5. The Uno has no pin left for a separate warning
LED, so A3 is the one alarm output. The analog light sensor uses A2, while the
headlight driver was moved to D13.

## Project tree

```text
RescueRover/
├── platformio.ini
├── README.md
├── include/
│   ├── config.hpp
│   ├── pins.hpp
│   └── sensors.hpp
└── src/
    ├── main.cpp
    └── sensors.cpp
```

`main.cpp` contains the normal Arduino flow: commands, motors, safety checks,
outputs, and telemetry. `sensors.cpp` contains the sensor-library adapters,
filtering, and scheduled reads.

## Required libraries

PlatformIO installs the sensor libraries listed in `platformio.ini`: DHT sensor
library 1.4.7, Adafruit MPU6050 2.2.9, Adafruit Unified Sensor 1.1.15, and
NewPing 1.9.7. `Wire` and `SoftwareSerial` come with the Arduino AVR framework.
The application still owns filtering, scheduling, safety checks, and telemetry.

The final checked build uses:

- Flash: 20,434 bytes of 32,256 bytes (63.3%)
- Static SRAM: 666 bytes of 2,048 bytes (32.5%)

That leaves 1,382 bytes for stack and runtime use. The MPU6050 library creates
its I2C adapter during setup; no application-level allocation occurs in the
main loop.

## Wiring and power

All modules, the Uno, motor driver logic, and motor supply must share a common
ground. Do not power the motors from the Uno 5 V pin. Use a motor-appropriate
battery/supply on the L298N motor supply input. Use a stable regulated 5 V logic
supply sized for the Uno, modules, MQ-135 heater, lights, and buzzer; the MQ-135
heater alone is a significant load.

### L298N

Remove the ENA and ENB jumper caps. Join ENA and ENB and connect the joined node
to D3. Connect IN1..IN4 to D4..D7. This saves one pin but both motor sides must
use the same PWM speed. If a side runs backward, first swap that motor's wires or
change `MOTOR_LEFT_REVERSED` / `MOTOR_RIGHT_REVERSED` in `config.hpp`.

For a real safety build, add a 10 kΩ pulldown from the joined ENA/ENB node to
ground. It holds the bridge disabled while the Uno is resetting and D3 has not
yet become an output. Similar gate/base pulldowns are recommended on transistor
drivers for the headlights and buzzer.

### HC-05 / TS-040

Connect HC-05 TXD to Uno D10. Uno D11 is 5 V logic and **must not directly drive
HC-05 RXD**. Use a divider, for example:

```text
Uno D11 ---- 1 kΩ ----+---- HC-05 RXD
                      |
                     2 kΩ
                      |
                     GND
```

This produces about 3.3 V at RXD. Connect VCC and GND according to the TS-040
breakout markings. STATE and EN are not used. The default data-mode speed is
9600 baud and can be changed with `BT_BAUD_RATE`.

### Sensors and outputs

- HC-SR04: VCC to 5 V, GND to ground, TRIG D8, ECHO D9.
- DHT11 module: OUT D12, plus its specified power and ground pins.
- MQ-135: AO A0; DO is unused. Do not draw its heater current through an
  undersized supply.
- LM393 microphone module: AO A1; DO is unused.
- HC-SR501: OUT D2. Its startup stabilization is 30 seconds by default.
- 4-pin light module: VCC to 5 V, GND to ground, AO to A2, and leave DO
  disconnected. The module's AO value is read as a 0..1023 ADC value.
- GY-521: SDA A4 and SCL A5. A normal GY-521 breakout is commonly powered from
  5 V through its onboard regulator; verify the markings and schematic of your
  exact board before applying power.
- Headlights: D13 must drive a logic-level MOSFET or transistor if the LEDs
  exceed safe GPIO current. Use LED current limiting. Do not power lamp current
  directly from an Uno pin.
- Active buzzer: D/A3. Use a transistor if its current exceeds the GPIO rating.
  A passive piezo is not a substitute for the configured active buzzer.

## Configuration and calibration

All editable thresholds, polarities, periods, speeds, and feature choices are
in `include/config.hpp`; all pins are in `include/pins.hpp`.

The supplied thresholds are starting values only:

- Measure clear-path HC-SR04 behavior and choose obstacle distances for the
  actual rover speed and stopping distance. Invalid readings block forward by
  default. A new close reading bypasses the three-sample median delay.
- Log MQ-135 raw and filtered values in clean air after stabilization. Set
  `GAS_BASELINE_ADC`, `GAS_WARNING_ABOVE_BASELINE`, and
  `GAS_CRITICAL_ABOVE_BASELINE`. The development warm-up is 60 seconds, but a
  new MQ sensor normally needs a much longer initial burn-in and each use needs
  several minutes of stabilization. Temperature, humidity, supply voltage, and
  sensor aging affect it. **The firmware reports ADC values, not calibrated ppm,
  and is not a professional gas analyzer.**
- Observe `SND` with the rover stationary in representative ambient noise and
  set `SOUND_THRESHOLD` above that noise floor. The module detects only acoustic
  level/activity. It does not recognize speech or the word “help.”
- Let the HC-SR501 stabilize, then tune its onboard sensitivity/time controls.
  PIR results are accepted only while stationary; a scan performed during its
  startup period reports no motion.
- Mount the MPU6050 rigidly and level. Adjust `MPU_PITCH_OFFSET_DEG` and
  `MPU_ROLL_OFFSET_DEG`, then test warning, critical, and rollover angles while
  supporting the rover. Accelerometer-only angles are affected by vehicle
  acceleration and vibration.
- With the rover in representative bright and dark conditions, observe
  `LRAW` and set `LIGHT_DARK_THRESHOLD` between the two readings. `LIGHT:1`
  means the firmware considers it dark. If the raw value increases in darkness,
  set `LIGHT_ANALOG_DARK_BELOW` to `0`.
- Compare DHT11 results with a reference meter before choosing temperature
  warnings. The DHT11 is low-resolution and is deliberately read no faster than
  every two seconds.

## Bluetooth protocol

Commands are single ASCII characters and are case-insensitive. Newlines are
ignored.

| Command | Action |
|---|---|
| `F` or `W` | Forward |
| `B` | Backward |
| `L` or `A` | Pivot left |
| `R` or `D` | Pivot right |
| `S` | Stop |
| `T` | Start scan |
| `X` | Immediate latched emergency stop |
| `C` | Clear the latch only after critical conditions are gone |
| `P` | Heartbeat; refresh the movement timeout |
| `H` | Force headlights on |
| `J` | Force headlights off |
| `U` | Return headlights to automatic mode |
| `0`..`9` | Select speed from minimum to full PWM |
| `?` | Print compact command help |

`S` always means stop, preserving the requested F/B/L/R/S safety protocol. A
WASD-style backward command is intentionally not accepted because it would make
`S` ambiguous.

A movement command authorizes motion for only 1000 ms. Repeat the direction or
send `P` before the timeout. Loss of traffic produces `ALERT:COMMAND_TIMEOUT`
and stops the motors. The HC-05 STATE pin is not required.

On macOS, pair the HC-05, locate its serial device with `ls /dev/tty.*`, and
connect with a serial terminal at 9600 baud. One simple option is:

```sh
screen /dev/tty.YOUR_HC05_DEVICE 9600
```

In `screen`, press Control-A then Control-Backslash to exit. The exact device
name depends on the module and macOS pairing.

## States, scan, and safety

The externally reported states are `STARTUP`, `IDLE`, `DRIVING`, `SCANNING`,
`WARNING`, and `EMERGENCY`. Warning/emergency conditions take display priority
over the underlying activity.

`T` stops the motors, waits 500 ms without blocking the main loop, opens a 50 ms
incremental sound window, evaluates PIR, then refreshes DHT when its two-second
limit allows, gas, distance, tilt, and light. It sends a `SCAN:` summary. A new
movement command cancels a scan.

Safety conditions are checked on every pass through `loop()` and before a new
movement command is accepted:

- A critical front obstacle stops forward motion and rejects new forward
  commands. Backward/pivot escape remains available.
- Missing/invalid ultrasonic data blocks forward motion by default.
- Critical gas or critical tilt stops and blocks every motor command while the
  condition exists.
- A rollover angle latches the emergency state.
- High temperature warns. Critical temperature stopping is available through
  `TEMP_CRITICAL_STOPS_MOTORS` and is off by default.
- Movement without a repeated command or heartbeat stops at the timeout.
- `X` immediately stops and latches. `C` cannot clear while gas, tilt, or an
  enabled temperature-stop condition remains critical.

## Telemetry

Telemetry is sent every 500 ms to the Bluetooth terminal. It uses no JSON or
formatting buffer:

```text
D:34,T:26.0,H:58.0,G:412,GR:420,GS:NORMAL,SND:72,LRAW:318,PIR:0,PITCH:4,ROLL:2,LIGHT:1,STATE:DRIVING
```

`G` is filtered MQ-135 ADC, `GR` is raw ADC, and `GS` is `WARM`, `NORMAL`,
`WARNING`, or `CRITICAL`. `NA` marks an invalid distance, DHT, or MPU reading.
Immediate messages include:

```text
ALERT:OBSTACLE
ALERT:GAS_CRITICAL
ALERT:SOUND_DETECTED
ALERT:MOTION_DETECTED
ALERT:TILT_CRITICAL
ALERT:COMMAND_TIMEOUT
```

## Build, upload, and terminal connection

1. Open this folder in VS Code with the PlatformIO extension installed.
2. Run **PlatformIO: Build**, or from the project terminal run `pio run`.
3. Connect the Uno and run **PlatformIO: Upload**, or `pio run -t upload`.
4. Pair the HC-05 with the computer and connect to it from a terminal at 9600
   baud as described in the Bluetooth protocol section.

No upload port is hard-coded; PlatformIO can discover the connected Uno. The
firmware does not print telemetry to USB; all runtime information goes to the
Bluetooth terminal.

## Controlled test procedure

Start with the rover raised so its wheels cannot touch anything. Keep the motor
supply off while checking sensors. Watch the Bluetooth terminal throughout.

1. **Motor directions:** connect a working HC-SR04 with clear space, power the
   motor stage, and send repeated `F`, `B`, `L`, `R`, and `S`. Confirm both sides
   and reverse a side in configuration if required. Test speed digits at low
   settings first.
2. **Bluetooth:** confirm `RESCUE_ROVER:READY`, send `?`, then verify each command
   and all telemetry from the Mac terminal.
3. **Command timeout:** send `F` once and no heartbeat. Verify stopping at about
   one second and `ALERT:COMMAND_TIMEOUT`.
4. **Ultrasonic stop:** at low wheel speed, move a flat target from more than 30
   cm to less than 10 cm. Verify warning, stop, and forward rejection; verify
   `B` can escape. Disconnect ECHO and verify forward is fail-safe blocked.
5. **DHT11:** breathe near (not onto) the sensor or move it between environments.
   Verify `T`/`H` update about every two seconds and failures show `NA`.
6. **MQ-135:** after warm-up, observe clean-air raw/filtered values. For a bench
   threshold test, temporarily set warning/critical values just above the
   baseline rather than exposing the sensor to dangerous gas. Verify critical
   gas stops all motors, then restore calibrated values.
7. **Sound:** keep motors stopped, clap near the microphone, and verify increased
   `SND` plus `ALERT:SOUND_DETECTED`. Confirm motor noise is not evaluated as a
   new stationary sound window while driving.
8. **PIR:** wait the full stabilization time, keep the rover stationary, move a
   warm body across its field, and verify `PIR:1` and the motion alert. Confirm
   it is suppressed while driving.
9. **Tilt:** with motors unpowered, slowly support and tilt the chassis through
   warning and critical angles. Verify the alerts and critical motor block. Test
   `C` only after returning below critical.
10. **Automatic headlights:** cover/uncover the LDR. Verify `LRAW`, `LIGHT`, and
    the lamps. If the analog direction is reversed, change
    `LIGHT_ANALOG_DARK_BELOW`; also test `H`, `J`, and `U`.
11. **Emergency stop:** while wheels are raised and turning slowly, send `X`.
    Verify immediate stop, rejected movement, `EMERGENCY`, and safe clearing by
    `C`.
12. **Scan mode:** send `T`, verify the motors stop, wait for the settle/sound
    phases, and confirm one `SCAN:` summary containing every sensor.
13. **Telemetry:** let the system run for several minutes. Confirm controlled
    500 ms updates, immediate alerts, no corrupted lines, and no unexplained
    resets.
14. **Integration:** power all sensors and motor logic together, initially with
    wheels raised. Exercise driving, heartbeat, obstacle stop, darkness, scan,
    tilt, gas threshold, and emergency stop. Then perform a low-speed floor test
    with a person ready to remove motor power.

## Known limitations and Uno compromises

- ENA and ENB share one PWM value, so independent left/right speed correction is
  unavailable. Steering is full pivot through direction control.
- The separate warning LED was dropped because the Uno has no remaining pin.
- One front ultrasonic sensor cannot see rear/side hazards; backward and pivot
  escape commands are not obstacle-protected.
- The HC-SR04 library call waits for an echo up to the configured 400 cm limit,
  and the DHT11 library read is blocking during its measurement. Both are
  scheduled infrequently; all other waiting, including scan settling and
  acoustic sampling, is scheduled without a long delay.
- At 9600 baud a complete SoftwareSerial telemetry line occupies the transmitter
  for tens of milliseconds. This is why output is rate-limited; command timeout
  still has ample margin, but commands should be repeated.
- MPU6050 angles use only acceleration and a small integer low-pass filter. They
  are approximate during acceleration, shock, or vibration.
- MQ-135, LM393 sound, PIR, DHT11, and threshold LDR modules are inexpensive
  indicators with substantial unit-to-unit variation. They cannot prove that a
  person is present or that an atmosphere is safe.
- There is no battery-voltage, motor-current, wheel-encoder, localization, or
  communications-link-quality measurement in the listed hardware.
