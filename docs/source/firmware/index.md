# Firmware

The ESP32 firmware is a PlatformIO project in `firmware/esp32/robot-firmware`, written in C++17 on
the Arduino-ESP32 framework (see {doc}`../development/adr/0002-cpp17-on-arduino-esp32`).

:::{toctree}
:maxdepth: 1

architecture
parameters
drivers
tuning
:::

## Project layout

```text
firmware/esp32/robot-firmware/
├── platformio.ini        envs: esp32 (robot), bringup (bring-up tool), native (host tests)
├── include/
│   ├── pins.hpp          board wiring, from the KiCad netlist
│   ├── board.hpp         wiring parameters resolved and checked at compile time
│   ├── robot_config.hpp  generated from config/robot_params.yaml
│   ├── credentials.hpp   picks secrets.hpp or the example fallback
│   └── secrets.example.hpp
├── lib/
│   ├── hal/              hardware interfaces (I2C, PWM, GPIO, counter, clock)
│   ├── hal_arduino/      ESP32 implementations of those interfaces
│   ├── hal_fakes/        test doubles for host unit tests
│   ├── drivers/          IMU, motor driver, encoder and mounting logic (portable)
│   ├── app/              ControlLoop, CommsService, task channels (portable)
│   ├── logging/          portable printf-style logger with pluggable sinks
│   ├── diagnostics/      portable I2C bus scan
│   └── telemetry/        WiFi SoftAP + telnet log sink and input (Arduino)
├── src/main.cpp          composition root: objects, FreeRTOS tasks
├── src/bringup/          interactive hardware bring-up tool (env "bringup")
└── test/                 GoogleTest suites run on the host
```

Libraries are split by their dependencies: anything that does not include Arduino headers is
compiled and unit-tested on the development machine; Arduino-specific code stays thin.

## Build, flash and test

```bash
cd firmware/esp32/robot-firmware
pio run -e esp32               # build
pio run -e esp32 -t upload     # flash (stop the Pi bridge first: UART0 is shared)
pio test -e native             # unit tests on the host
pio check -e esp32             # cppcheck + clang-tidy
```

## Credentials

WiFi credentials live in `include/secrets.hpp`, which is git-ignored. Copy the template and edit
it:

```bash
cp include/secrets.example.hpp include/secrets.hpp
```

Without `secrets.hpp` the build still succeeds and uses the placeholder values from the example
(the compiler prints a note). This keeps fresh clones and CI building.

## Debug log access

UART0 is reserved for the link to the Raspberry Pi, so the firmware never writes logs to
`Serial`. Instead it opens a WiFi access point and streams its log over telnet:

1. Join the access point (default SSID `robot-debug`, see `secrets.hpp`).
2. Run `telnet 192.168.4.1 23`.

Each line starts with a three-letter level tag (`DBG`, `INF`, `WRN`, `ERR`). Only one telnet client
is served at a time.

Log lines also travel to the Pi as protocol log frames, so they show up in ROS 2 as well.

## Runtime structure

The firmware runs a 200 Hz control task and a communication task on separate cores; see
{doc}`architecture` for tasks, timing, data flow and failure handling, and {doc}`tuning` for
bringing up the controller on the robot.

## Coding conventions

- `.hpp`/`.cpp` only, `#pragma once`, everything in the `robot::` namespace.
- Fixed-width integer types from `<cstdint>`; configuration as `inline constexpr`.
- No heap allocation and no Arduino `String` outside start-up code.
- printf-style logging is format-checked at compile time (`ROBOT_PRINTF_FORMAT`).
- cppcheck and clang-tidy (including the C++ Core Guidelines checks) must report no
  medium- or high-severity findings; every disabled check is justified in `.clang-tidy`.
