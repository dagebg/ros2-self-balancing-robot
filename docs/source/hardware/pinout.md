# Pinout

This table is the single reference for every signal between the ESP32 and the rest of the robot.
It was derived from the KiCad netlist, not from the drawing, so it reflects real electrical
connectivity.

## ESP32 DevKitC

| Function | ESP32 GPIO | Connected to | Notes |
|---|---|---|---|
| Motor A PWM | 25 | TB6612 `PWMA` | LEDC PWM |
| Motor A direction 1 | 26 | TB6612 `AIN1` | |
| Motor A direction 2 | 27 | TB6612 `AIN2` | |
| Motor B PWM | 32 | TB6612 `PWMB` | LEDC PWM |
| Motor B direction 1 | 33 | TB6612 `BIN1` | |
| Motor B direction 2 | 13 | TB6612 `BIN2` | |
| Motor driver standby | 16 | TB6612 `STBY` | Low = both outputs off |
| Encoder J5 (motor M2) phase A | 34 | J5 `E-A` | Input-only pin, no internal pull-up |
| Encoder J5 (motor M2) phase B | 4 | J5 `E-B` | |
| Encoder J6 (motor M1) phase A | 36 | J6 `E-A` | Input-only pin, no internal pull-up |
| Encoder J6 (motor M1) phase B | 35 | J6 `E-B` | Input-only pin, no internal pull-up |
| I2C SDA | 21 | MPU-9250 `SDA`, MPU-6050 `SDA` | 400 kHz |
| I2C SCL | 22 | MPU-9250 `SCL`, MPU-6050 `SCL` | 400 kHz |
| UART0 TX | 1 | Pi GPIO15 (`RXD`, header pin 10) | Shared with the USB-serial chip |
| UART0 RX | 3 | Pi GPIO14 (`TXD`, header pin 8) | Shared with the USB-serial chip |

## I2C devices

| Device | Address | Strap |
|---|---|---|
| MPU-9250 | `0x69` | AD0 tied to 3V3 |
| MPU-6050 | `0x68` | AD0 tied to GND |

## Free pins

GPIO 0, 2, 5, 12, 14, 15, 17, 18, 19, 23 and 39 are not connected. GPIO 0, 2, 5, 12 and 15 are
strapping pins and should stay unused unless their boot-time level is understood.

## Regenerating the netlist

KiCad ships a CLI (on macOS it lives inside the app bundle):

```bash
kicad-cli sch export netlist --format kicadsexpr \
  -o schematics.net mechanical_hardware/schematics/schematics.kicad_sch
```

Each `(net …)` block in the output lists the connected `(ref, pin)` pairs.
