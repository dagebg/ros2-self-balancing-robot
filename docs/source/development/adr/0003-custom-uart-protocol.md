# ADR-0003: Use a custom framed binary protocol over UART between ESP32 and Pi

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The ESP32 must receive velocity targets and send telemetry (IMU, encoders, controller state) to the
Raspberry Pi. The hardware already connects ESP32 UART0 (GPIO 1/3) to the Pi's UART (GPIO 14/15).
UART0 is also the USB-serial port, and the ESP32 boot ROM writes text to it at reset.

Requirements: low and predictable latency, robustness against line noise and boot output,
modest RAM use, and logic that can be unit-tested on both ends.

## Decision

Use a small custom binary protocol on the existing UART link:

- Each message is a packed, little-endian payload with a version byte, a message type and a
  sequence number.
- A CRC-16/CCITT-FALSE checksum is appended, the result is COBS-encoded, and a `0x00` byte
  delimits frames. Any corrupted or partial frame is dropped and the parser resynchronizes at the
  next delimiter.
- The codec is one C++17 library, compiled into both the firmware and the ROS 2 bridge node.
- Because UART0 is reserved for this link, firmware logging goes to WiFi telnet and is also sent
  as protocol log frames.

## Alternatives considered

- **micro-ROS over UART:** the ESP32 would become a native ROS 2 node. Rejected for now because of
  the build complexity, the RAM cost of the XRCE-DDS client, and the difficulty of unit-testing.
  It also hides the timing of the link.
- **micro-ROS over WiFi (UDP):** no wiring needed, but WiFi latency jitter is a poor fit for a
  balancing robot's command path.
- **Text protocol (e.g. JSON lines):** easy to debug, but larger, slower to parse and without
  integrity checking.

## Consequences

- A protocol specification must be maintained and versioned.
- Both ends share one tested implementation, so encoding mismatches are caught by unit tests.
- Flashing over USB requires the Pi side to be idle (see {doc}`../../hardware/known-issues`).
