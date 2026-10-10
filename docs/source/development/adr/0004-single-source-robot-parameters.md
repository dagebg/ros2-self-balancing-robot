# ADR-0004: Keep robot parameters in one YAML file and generate the firmware header

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

Wheel radius, track width, gear ratio, encoder resolution, loop rates and the UART baud rate are
needed by the firmware (C++), the ROS 2 bridge and robot description (YAML/xacro) and the
simulation. Most of the physical values are not measured yet and will change. Copies of the same
constant in several languages drift apart silently, and a wrong wheel radius in one place shows up
only as subtly wrong odometry.

## Decision

- `config/robot_params.yaml` is the only place these constants are defined. Each entry carries a
  value, a unit, a description and a `measured` flag.
- `tools/gen_config.py` generates `include/robot_config.hpp` (C++17 `inline constexpr` values) for
  the firmware. The header is committed so the firmware builds without Python.
- A pre-commit hook and CI run the generator in `--check` mode and fail if the header is stale.
- ROS 2 and the simulation load the YAML directly; the docs render it as a table.
- The firmware warns at boot while any value is still a placeholder.

## Alternatives considered

- **Hand-maintained constants per component:** simplest, but drift is guaranteed.
- **Generate at build time only (not committed):** keeps the repository smaller but makes the
  firmware build depend on Python and hides the effective values from code review.
- **Runtime parameters sent from the Pi:** flexible, but the firmware must be able to balance
  without the Pi, so its constants must be compiled in.

## Consequences

- Changing a constant is a one-line YAML edit plus regeneration, reviewable in one diff.
- A small Python tool with its own tests becomes part of the toolchain.
- Placeholders are visible everywhere (header comments, docs table, boot log) until measured.
