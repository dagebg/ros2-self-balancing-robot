# ADR-0008: Decode wheel encoders with the ESP32 PCNT peripheral

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

Each N20 motor has a two-phase Hall encoder on the motor shaft, before the gearbox. At full speed
this produces several thousand edges per second per wheel. The encoder inputs sit on GPIO 34, 35,
36 and 4; GPIO 36 is affected by an ESP32 erratum that can produce short spurious low pulses while
the ADC or WiFi is active.

## Decision

Decode both encoders in hardware with the pulse counter (PCNT), one unit per wheel, using two
channels for ×4 quadrature decoding and the PCNT glitch filter (1.25 µs). The control loop reads
the 16-bit hardware count each cycle and `CounterExtender` builds a 32-bit total from it.

## Alternatives considered

- **GPIO interrupts per edge:** thousands of interrupts per second on the same core as WiFi and
  the control loop add jitter, and spurious pulses on GPIO 36 would be counted.
- **ESP32Encoder library:** wraps the same PCNT hardware, but adds a dependency for about
  40 lines of configuration and hides the filter settings.

## Consequences

- Encoder decoding costs no CPU time and no interrupts.
- The 16-bit counter must be read at least every ~15000 counts; the 200 Hz control loop reads it
  every 5 ms, orders of magnitude more often than necessary.
- The PCNT configuration uses the ESP-IDF 4.4 legacy driver (`driver/pcnt.h`); moving to
  Arduino-ESP32 3.x / ESP-IDF 5 will require porting it to the new `pulse_cnt` API.
