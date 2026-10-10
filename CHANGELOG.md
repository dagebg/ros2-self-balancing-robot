# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Sphinx documentation with Markdown (MyST), Mermaid diagrams and Doxygen API reference.
- Hardware reference: bill of materials, netlist-derived pinout, power rails and known issues.
- Architecture decision records (ADR-0001 to ADR-0003).
- Contribution guide and this changelog.
- pre-commit hooks (ruff, clang-format, file hygiene) shared by local commits and CI.
- Dependabot for GitHub Actions and Python tools, a pull request template and `.editorconfig`.
- Documentation of the development environment, testing strategy and CI/CD pipeline.
- Firmware: portable logger with pluggable sinks, telnet log sink, I2C wiring check at boot and
  `pins.hpp` with the netlist-derived pin map.
- Firmware host unit tests with GoogleTest (`pio test -e native`) and a CI job that tests, builds,
  size-reports and statically analyzes the firmware.
- `config/robot_params.yaml` as the single source of robot constants, with a generator for the
  firmware header, tests, a pre-commit check and a docs table (ADR-0004, ADR-0005).
- `shared/robot_protocol`: COBS + CRC-16 framed binary protocol v1 with eight message types, a
  resynchronizing stream parser and loss tracking, built by CMake, colcon and PlatformIO from one
  source (ADR-0006). 53 tests including golden wire bytes, reference vectors and seeded fuzzing.
- CI job for shared libraries: coverage (≥ 90 % gate) with clang-tidy, and an ASan/UBSan run.
- Protocol specification with byte layouts, validation rules, link budget and versioning policy.
- Firmware hardware abstraction (I2C, PWM, GPIO, quadrature counter, clock) with ESP32
  implementations and host test fakes.
- Drivers: one MPU-6050/MPU-9250 driver (200 Hz, 42 Hz DLPF, ±4 g, ±500 °/s), TB6612 H-bridge
  with duty limit and NaN-safe commands, PCNT ×4 encoder decoding with glitch filter and 32-bit
  extension, IMU mounting transform (ADR-0007, ADR-0008).
- Wiring and IMU-mounting parameters with compile-time validation in `board.hpp`.
- Interactive bring-up firmware (`pio run -e bringup`) and a step-by-step bring-up guide.
- `shared/balance_core`: complementary filter with accelerometer rejection, gyro-bias calibration,
  dual-IMU fusion with disagreement detection, PID with derivative on measurement and anti-windup,
  cascaded velocity/pitch/yaw-rate controller, arming and fault state machine, and odometry that
  compensates for body pitch (ADR-0009).
- Wheeled-inverted-pendulum plant model and closed-loop tests: balancing, push recovery, velocity
  and yaw tracking, and robustness of the default gains across ±30–40 % parameter errors.
- Balance control documentation with the estimator, plant equations, controller and safety logic.
- Firmware application: a 200 Hz control task on core 1 and a communication task on core 0,
  connected by lock-free queues and latest-value mailboxes, with a 1 s task watchdog (ADR-0010).
- `ControlLoop` (per-IMU bias calibration while disarmed, fusion, estimation, safety, odometry and
  control) and `CommsService` (protocol routing, acknowledgements, 50 Hz telemetry, log
  forwarding to the Pi), both covered by host tests including a firmware-level closed loop.
- Link supervision: without frames from the Pi for 500 ms the robot stops but keeps balancing.
- Firmware architecture page and a calibration and tuning guide.
- ROS 2 packages `robot_interfaces` (`BalanceState`) and `robot_bridge`: cmd_vel forwarding with
  heartbeat and timeout, balance state, pitch-compensated odometry with TF, joint states, both
  IMUs, diagnostics, firmware logs in `/rosout`, an `~/arm` service answered by the ESP32's
  acknowledgement, and live PID gains as validated parameters synced on every connection.
- Bridge tests: conversions, the serial port over a pseudo-terminal, and the running node against
  a fake ESP32.
- `tools/ros-docker.sh` and `docker/dev.Dockerfile`: reproducible ROS 2 Jazzy builds and tests on
  any machine; CI job that builds and tests all ROS 2 packages in `ros:jazzy`.
- Raspberry Pi setup guide for the PL011 UART.
- `robot_description`: URDF generated from `robot_params.yaml` (installed with the package), with
  the body pitch as a joint so RViz shows the measured lean, a free-floating variant for
  simulation, an RViz preview and model tests.
- `robot_bringup`: one launch file for model, bridge, rosbridge, optional gamepad teleoperation and
  RViz; parameters derived from `robot_params.yaml`; a launch test that starts it without hardware.
- `wheel_mass` robot parameter.
- `robot_sim`: Gazebo Harmonic simulation with a virtual ESP32 that runs the firmware's
  `ControlLoop` and `CommsService` behind the unchanged bridge and bringup over a pseudo-terminal,
  with a DC-motor model and automatic arming after calibration (ADR-0011).
- End-to-end simulation test in CI: telemetry, arming through the bridge, 8 s of balancing and
  driving forward, checked against Gazebo's ground truth.
- Gazebo extensions of the robot model (IMU, wheel friction, joint states, torque inputs, ground
  truth) for `sim:=true`.
- Configurable gyro calibration window in `ControlConfig`.
- Web app (React, TypeScript, Vite) over rosbridge: status and faults, arming, joystick and keyboard
  driving, live PID tuning through the bridge's parameter services, and a telemetry plot
  (ADR-0012). Unit and component tests against an in-memory client, an opt-in integration test
  against the running simulation, and a CI job (lint, typecheck, tests, build).
- `ROS_DOCKER_ARGS` for `tools/ros-docker.sh` (e.g. to publish rosbridge's port).
- Docker images for the Pi: `robot` (arm64, built natively on an Arm runner) and `web` (nginx,
  multi-architecture), pushed to GHCR from `main` and release tags, and a `docker compose` file for
  the Pi (ADR-0013).
- Release workflow: tag `vX.Y.Z` publishes merged firmware and bring-up flash images, the firmware
  ELF, the web app bundle and git-cliff release notes.
- `tools/merge_firmware.sh` for single-file flash images; deployment and release guides.

### Changed (ROS 2)

- `robot_bridge` publishes odometry for `base_footprint` and the body pitch in `joint_states`.
- The development image and CI skip GUI-only dependencies (RViz, joint sliders).

### Changed

- CI restructured: a single `lint` job runs pre-commit, all tool versions are pinned and cached,
  and superseded runs are cancelled.
- `.clang-format` moved to the repository root so one C++ style applies everywhere.
- Firmware converted to modern C++17 (`.hpp`, namespaces, `constexpr` configuration, no Arduino
  `String`); `WifiDebug` replaced by `robot::telemetry::TelnetSink`; platform pinned to
  `espressif32@7.1.3`. clang-tidy now includes the C++ Core Guidelines checks.
- Firmware logs no longer go to `Serial`, which is reserved for the Raspberry Pi link.
- The I2C bus scan now runs once at start-up only, since the control task owns the bus afterwards.

### Fixed

- Documentation build warnings, broken links and leftover citation markers.
- Documentation is now built on every pull request, with warnings treated as errors.
- Firmware failed to build from a fresh clone: `SECRETS.h.example` defined different macro names
  than `main.cpp` used. Credentials now fall back to `secrets.example.hpp` automatically.
