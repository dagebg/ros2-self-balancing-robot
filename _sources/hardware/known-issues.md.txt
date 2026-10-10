# Known hardware issues

Findings from reviewing the KiCad netlist against the firmware's needs. Each item states its impact
and how the software deals with it until the hardware is changed.

## Motor outputs not connected in the schematic

The TB6612FNG outputs `AO1`, `AO2`, `BO1` and `BO2` are drawn, but in the netlist they are not
connected to the motor connectors J5/J6 (`M+`/`M-`).

- **Impact:** the schematic does not document which driver channel turns which wheel, or in which
  direction.
- **Mitigation:** channel-to-wheel mapping and direction inversion are configuration values, and
  the bring-up firmware spins each channel separately so they can be determined on the robot.
- **Fix:** connect the outputs to J5/J6 in KiCad.

## MPU-9250 `VDDIO` unconnected

Pin 8 (`VDDIO`) of the MPU-9250 symbol has no connection. Most breakout boards tie it to `VDD`
on-board, so this is likely a schematic-only omission. Verify against the breakout board.

## Encoder inputs on input-only pins

GPIO 34, 35 and 36 are input-only and have no internal pull-up or pull-down resistors. This is fine
for Hall encoders with push-pull outputs; open-collector encoders would need external pull-ups.

GPIO 36 (`SENSOR_VP`) is also affected by an ESP32 erratum: it can see short spurious low pulses
when the ADC or WiFi is active. The firmware decodes encoders with the PCNT peripheral and its
glitch filter, which rejects pulses that short.

## UART0 is shared between USB and the Pi

ESP32 UART0 (GPIO 1/3) is connected both to the DevKit's USB-serial chip and to the Pi.

- While flashing over USB, the Pi's TX line drives the same RX pin. Stop the ROS bridge (or
  unplug the Pi link) before flashing.
- The ESP32 boot ROM prints messages on UART0 at reset. The protocol's framing and CRC discard
  this noise.
- Firmware logs therefore never go to `Serial`; they go over WiFi telnet and as protocol log
  frames.

## No battery-voltage sensing

No battery voltage reaches an ADC pin, so the firmware cannot detect a low battery. A voltage
divider from the battery to a free ADC1 pin (for example GPIO 39) would fix this.
