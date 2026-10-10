# ADR-0007: Write a small IMU driver instead of using third-party libraries

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The first IMU experiments used two Arduino libraries: `electroniccats/MPU6050` and
`bolderflight/MPU9250`. They bring two different APIs, unit conventions and configuration styles
for chips that share the same accelerometer/gyroscope register map. Both talk to `Wire` directly,
so they cannot run in host unit tests. The balance controller needs only one configuration:
200 Hz, a ~42 Hz low-pass filter and fixed ranges.

## Decision

Implement one `Mpu6xxx` driver (about 100 lines) on top of the `hal::I2cBus` interface. It
covers both chips through a variant parameter, verifies WHO_AM_I before writing, configures
exactly what the controller needs and returns SI units. The MPU-9250's magnetometer is left out
until yaw fusion is actually needed.

## Alternatives considered

- **Keep both libraries:** less code to write, but two APIs, no host tests, and configuration
  hidden in library defaults (one example ran the MPU-9250 at 50 Hz).
- **A single larger library (e.g. I2Cdevlib):** still untestable without hardware and much more
  surface than needed.

## Consequences

- The register sequence is pinned by unit tests against a fake I2C bus.
- Supporting other chips or features (FIFO, interrupts, magnetometer) means extending our own
  code; the datasheet references are in the source.
