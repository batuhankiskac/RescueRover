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
| D0 | HC-05 STATE | Digital input; HIGH means connected |
| D1 | USB / upload TX | Kept free for uploading |
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
| A3 | Active buzzer | Digital output; critical-event alarm only |
| A4 | MPU6050 SDA | I2C |
| A5 | MPU6050 SCL | I2C |

There are no pin conflicts. SoftwareSerial uses D10/D11, Timer 2 PWM is used on
D3, and the MPU6050 owns A4/A5. The Uno has no pin left for a separate warning
LED, so A3 is the critical-event buzzer output. The analog light sensor uses A2,
while the headlight driver was moved to D13. D0 is reserved for the HC-05 STATE input;
D1 remains available for the USB upload/serial TX function.

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

- Flash: 20,864 bytes of 32,256 bytes (64.7%)
- Static SRAM: 673 bytes of 2,048 bytes (32.9%)

That leaves 1,375 bytes for stack and runtime use. The MPU6050 library creates
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
yet become an output. The firmware also preloads all motor outputs LOW and
rejects motion commands during its first two seconds, but software cannot
control pins while the Uno is still in reset. Similar gate/base pulldowns are
recommended on transistor drivers for the headlights and buzzer.

### HC-05 / TS-040

Connect HC-05 TXD to Uno D10 and HC-05 STATE to Uno D0. STATE is HIGH while the
module is connected and LOW while it is disconnected. Uno D11 is 5 V logic and
**must not directly drive HC-05 RXD**. Use a divider, for example:

```text
Uno D11 ---- 1 kΩ ----+---- HC-05 RXD
                      |
                     2 kΩ
                      |
                     GND
```

This produces about 3.3 V at RXD. Connect VCC and GND according to the TS-040
breakout markings. STATE is connected to D0; EN is not used. The default data-mode speed is
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

All editable thresholds, polarities, periods, and feature choices are in
`include/config.hpp`; all pins are in `include/pins.hpp`. Motor speed remains
the fixed `DEFAULT_MOTOR_SPEED` value.

The supplied thresholds are starting values only:

- Measure clear-path HC-SR04 behavior and choose obstacle distances for the
  actual rover speed and stopping distance. At startup, three valid readings
  are required before forward motion is allowed. After validation, no echo is
  treated as an open path beyond range and remains visible as `D:NA`. A new
  close reading bypasses the three-sample median delay.
- Log MQ-135 raw and filtered values in clean air after stabilization. Set
  `GAS_BASELINE_ADC`, `GAS_WARNING_ABOVE_BASELINE`, and
  `GAS_CRITICAL_ABOVE_BASELINE`. The development warm-up is 60 seconds, but a
  new MQ sensor normally needs a much longer initial burn-in and each use needs
  several minutes of stabilization. Temperature, humidity, supply voltage, and
  sensor aging affect it. **The firmware reports ADC values, not calibrated ppm,
  and is not a professional gas analyzer.**
- Observe `SND` with the rover stationary in representative ambient noise and
  set `SOUND_THRESHOLD` above that noise floor. The module detects only acoustic
  level/activity; it does not recognize speech or the word “help.”
- Let the HC-SR501 stabilize, then tune its onboard sensitivity/time controls.
  PIR results are accepted only while stationary; a scan performed during its
  startup period reports no motion. `PIR:1` remains available in telemetry.
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
  every two seconds. Two consecutive valid readings at or above 60 °C are required
  to enter critical temperature, and two consecutive valid readings below 60 °C
  are required to clear it. Invalid reads never clear an active critical lock.

## Bluetooth protocol

Commands are single ASCII characters and are case-insensitive. Newlines are
ignored.

| Command | Action |
|---|---|
| `W` | Forward |
| `S` | Backward |
| `A` | Pivot left |
| `D` | Pivot right |
| `Space` | Stop |
| `T` | Start scan |
| `X` | Immediate latched emergency stop |
| `C` | Clear the latch only after critical conditions are gone |
| `P` | Heartbeat; refresh the movement timeout |
| `?` | Print compact command help |

Movement uses only WASD. The old `F`, `B`, `L`, and `R` movement commands are
ignored. Press Space for a normal stop or `X` for the latched emergency stop.

A movement command authorizes motion for only 1000 ms. Repeat the direction or
send `P` before the timeout. Loss of traffic produces `ALERT:COMMAND_TIMEOUT`
and silently stops the motors. The firmware also stops and rejects movement
whenever the HC-05 STATE input is LOW. If the exact module uses inverted STATE
logic, change `BT_STATE_ACTIVE_HIGH` in `include/config.hpp`. Headlights are
always controlled automatically by the light sensor; there are no manual
headlight commands.

On macOS, pair the HC-05, locate its serial device with `ls /dev/tty.*`, and
connect with a serial terminal at 9600 baud. One simple option is:

```sh
screen /dev/tty.YOUR_HC05_DEVICE 9600
```

In `screen`, press Control-A then Control-Backslash to exit. The exact device
name depends on the module and macOS pairing.

## States, scan, and safety

Telemetry separates the operating state from safety severity. `STATE` is one of
`STARTUP`, `IDLE`, `DRIVING`, `SCANNING`, or `LOCKED`. `LEVEL` is one of
`NORMAL`, `WARNING`, `ALERT`, or `CRITICAL`, with the highest active condition
winning. Transient events such as command timeout are carried by their immediate
message and do not remain latched in `LEVEL`.

Immediate message prefixes are real severity levels: `WARNING` is informational,
`ALERT` requires attention or reports a protective action, and `CRITICAL` marks
an immediate hazard. Non-hazard acknowledgements use `STATUS`. If several
leveled events arise in the same loop pass, only one message at the highest
severity is sent; ties keep the first detected event.

| Condition | Reported severity | Motor action | Buzzer |
|---|---|---|---|
| PIR motion without a nearby obstacle | `WARNING` | None | Silent |
| Sound detected | `ALERT` | None | Silent |
| Ultrasonic not validated after startup | `ALERT` | Forward blocked | Silent |
| Validated ultrasonic reports `D:NA` | `NORMAL` | None | Silent |
| Obstacle at 11..30 cm | `WARNING` | None | Silent |
| Obstacle at 10 cm or less while not advancing | `ALERT` | Forward blocked | Silent |
| Obstacle at 10 cm or less while advancing/trying `W` | `CRITICAL` | Forward stopped/blocked | One 250 ms beep |
| Gas, temperature, or tilt warning threshold | `WARNING` | None | Silent |
| Gas, confirmed temperature, or tilt critical threshold | `CRITICAL` | All motion blocked | One 250 ms beep |
| Rollover or `X` emergency stop | `CRITICAL` | All motion latched | One 250 ms beep |
| Command timeout | `ALERT` | Current motion stopped | Silent |

The table is the normative event/action mapping. Critical events selected in
the same event cycle, or arriving during an active beep, share that beep; it is
never extended or queued. A source beeps again only after clearing and re-entry.

`T` stops the motors, waits 500 ms without blocking the main loop, opens a 50 ms
incremental sound window, evaluates PIR, then refreshes DHT when its two-second
limit allows, gas, distance, tilt, and light. It sends a `SCAN:` summary. A new
movement command cancels a scan.

Safety is checked on every loop and before accepting motion. Backward and pivot
escape remain available during a front-obstacle block; collision risk rearms
only after the target leaves the 30 cm warning zone. Ultrasonic validation still
requires three valid startup readings, while post-validation `D:NA` is treated
as open path beyond range. Invalid DHT/MPU samples never clear active temperature
or tilt critical state. Rollover and `X` remain latched until a safe `C`; active
gas, temperature, tilt, or an invalid rollover reading rejects that command.

## Telemetry

Telemetry is sent every 500 ms to the Bluetooth terminal. It uses no JSON or
formatting buffer:

```text
D:34,T:26.0,H:58.0,G:412,GR:420,GS:NORMAL,SND:72,LRAW:318,PIR:0,PITCH:4,ROLL:2,LIGHT:1,STATE:DRIVING,LEVEL:NORMAL
```

`G` is filtered MQ-135 ADC, `GR` is raw ADC, and `GS` is `WARM`, `NORMAL`,
`WARNING`, or `CRITICAL`. `NA` marks an invalid distance, DHT, or MPU reading.
Immediate messages include `WARNING:OBSTACLE`, `WARNING:MOTION_DETECTED`,
`ALERT:OBSTACLE_CLOSE`, `ALERT:SOUND_DETECTED`, `ALERT:COMMAND_TIMEOUT`,
`CRITICAL:COLLISION_RISK`, `CRITICAL:GAS`, `CRITICAL:TILT`, and
`STATUS:EMERGENCY_CLEARED`.

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
supply off while checking sensors and watch Bluetooth throughout. These physical
tests are still pending; a successful build does not validate sensor behavior.

1. **Controls and Bluetooth:** confirm `RESCUE_ROVER:READY`, help, and telemetry.
   With the wheels raised and a clear path, test repeated `W/S/A/D` commands and
   Space; confirm both motor sides before any floor test.
2. **Command timeout:** send `W` once without another command or heartbeat.
   Confirm a silent stop and `ALERT:COMMAND_TIMEOUT` after about one second.
3. **Ultrasonic:** verify silent warning at 20 cm and silent alert at 5 cm. Send
   `W` at 5 cm and require one collision-risk beep plus forward rejection while
   `S/A/D` remain available. Repeated `W` stays silent until the target passes
   30 cm. After startup validation, disconnect ECHO and confirm silent `D:NA`
   with forward motion available.
4. **PIR:** after stabilization, test distant motion while stationary and verify
   `PIR:1` plus `WARNING:MOTION_DETECTED`. Repeat with a target inside 30 cm:
   `PIR:1` remains but the duplicate motion message is suppressed. Confirm PIR
   events are suppressed while driving.
5. **DHT11:** verify `T/H` updates and invalid readings. For a safe bench test,
   temporarily move both temperature thresholds around the measured room value,
   keeping warning below critical. Require two valid high readings to lock and
   beep, two valid safe readings to clear, and no unlock on an invalid reading;
   then restore the configured thresholds.
6. **MQ-135:** after warm-up, observe clean-air raw and filtered values. Use
   temporary bench thresholds rather than dangerous gas; verify silent warning,
   one critical beep, full motor block, and restoration of calibrated values.
7. **Sound:** while stationary, clap near the microphone and confirm increased
   `SND` with silent `ALERT:SOUND_DETECTED`. Confirm driving noise does not start
   a new stationary sound event.
8. **Tilt and rollover:** with motor power off, cross warning, critical, and
   rollover angles. Confirm silent warning, one critical beep, motor block,
   invalid-MPU lock retention, and latched rollover. Test `C` only after a valid
   below-critical reading.
9. **Emergency stop:** while raised-wheel motion is active, send `X` and verify
   immediate stop, one beep, locked telemetry, rejected movement, and safe `C`
   acknowledgement through `STATUS:EMERGENCY_CLEARED`.
10. **Outputs and scan:** cover and uncover the LDR to verify `LRAW`, `LIGHT`, and
    headlights. Send `T` and confirm one complete `SCAN:` result after settling.
11. **Telemetry and integration:** run every sensor, driving, heartbeat, scan,
    and emergency case for several minutes. Require 500 ms `STATE`/`LEVEL`
    telemetry, correct event prefixes, and no corruption or unexplained resets;
    then perform a low-speed floor test with someone ready to cut motor power.

## Known limitations and Uno compromises

- ENA and ENB share one PWM value, so independent left/right speed correction is
  unavailable. Steering is full pivot through direction control.
- The separate warning LED was dropped because the Uno has no remaining pin.
- One front ultrasonic sensor cannot see rear/side hazards; backward and pivot
  escape commands are not obstacle-protected.
- The front PIR and ultrasonic sensor cannot prove that they see the same object.
  PIR remains stationary-only context; a current ultrasonic obstacle within
  30 cm suppresses only the duplicate PIR message, never the distance safety
  decision or `PIR` telemetry value.
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
