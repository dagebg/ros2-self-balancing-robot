# ADR-0002: Write the firmware in modern C++17 on the Arduino-ESP32 framework

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The first firmware mixed C and C++ styles: `.h` headers with include guards, free functions over
hidden global state, C strings next to Arduino `String`, and `#define` constants. That mix makes
ownership unclear, hampers unit testing and does not match the C++ used on the ROS 2 side.

The firmware needs a fixed-rate control loop, I2C, PWM, hardware pulse counters, WiFi and UART.
It should also be testable on a development machine without the robot.

## Decision

- All firmware code is C++17: `.hpp`/`.cpp` files, `#pragma once`, code in the `robot::`
  namespace, `<cstdint>` types, `constexpr` configuration and `enum class`.
- No Arduino `String` and no heap allocation in the control path.
- Keep the Arduino-ESP32 framework (built with PlatformIO) for its drivers and libraries, but
  confine it to a thin hardware abstraction layer. Everything above that layer is plain C++ and is
  compiled and unit-tested on the host.
- The ESP-IDF APIs bundled with Arduino-ESP32 (FreeRTOS, PCNT) are used directly where Arduino has
  no equivalent.

## Alternatives considered

- **Pure C11 on ESP-IDF:** maximum control and very common in industry, but slower to develop.
  It would also need its own I2C, WiFi and PWM glue, and still uses `.h` headers.
- **C++ on bare ESP-IDF:** a clean toolchain, but it would mean rewriting working Arduino-based
  code and a heavier setup, for little gain in a project of this size.

## Consequences

- Core logic (control, estimation, protocol) is portable and shared with the ROS 2 and simulation
  code.
- The Arduino layer must stay thin; any direct ESP-IDF use is isolated in the hardware layer.
- The toolchain must be switched from the framework default (`gnu++11`) to `gnu++17`.
