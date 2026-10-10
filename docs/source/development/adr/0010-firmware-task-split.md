# ADR-0010: Split the firmware into a real-time control task and a communication task

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The balance loop must run every 5 ms with little jitter. The same firmware also serves a UART
protocol, a telnet log stream over WiFi and statistics. Arduino's single `loop()` would put all of
this on one timeline: a slow telnet write or a burst of incoming frames would delay the
controller. The logic should also stay testable without FreeRTOS.

## Decision

- A **control task** pinned to core 1 at high priority runs at a fixed rate with
  `vTaskDelayUntil`. It reads sensors, runs `ControlLoop`, drives the motors and never logs or
  touches WiFi.
- A **comms task** on core 0 (next to the WiFi stack) runs `CommsService`: protocol I/O,
  telemetry, acknowledgements, log forwarding.
- Tasks exchange data only through small typed channels: latest-value `Mailbox`es for state,
  lock-free `SpscQueue`s for commands and acknowledgements that must not be lost.
- All logic lives in portable classes; FreeRTOS appears only in `src/main.cpp`.
- The control task is subscribed to the task watchdog with a 1 s timeout.

## Alternatives considered

- **Single Arduino loop with timers:** simple, but every blocking call anywhere adds jitter to the
  balance loop.
- **Hardware timer interrupt for control:** lowest jitter, but I2C transactions and floating-point
  work do not belong in an ISR on the ESP32.
- **FreeRTOS queues for everything:** works, but ties the logic to FreeRTOS and makes host tests
  harder; the custom channels are a few lines each and fully tested.

## Consequences

- The control loop's timing is independent of link and WiFi load; its longest tick is reported in
  telemetry.
- Every cross-task value has an explicit type and direction, which documents the data flow.
- Two tasks mean two stacks to size; both are measured with headroom before release.
