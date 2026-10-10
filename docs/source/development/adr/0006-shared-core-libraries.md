# ADR-0006: Share hardware-independent C++ libraries between firmware, ROS 2 and simulation

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

Several pieces of logic must behave identically on the ESP32 and on the Raspberry Pi or a
development machine:

- the protocol codec, used by the firmware and the ROS 2 bridge;
- the balance controller, state estimation and kinematics, used by the firmware and the
  Gazebo simulation.

Each piece is built by a different system: PlatformIO for the firmware, colcon/ament for ROS 2,
and plain CMake for fast local tests with coverage and sanitizers.

## Decision

- Hardware-independent C++17 libraries live in `shared/<library>/` with no dependency on Arduino,
  ESP-IDF or ROS.
- Each library ships three thin build descriptions for the same sources:
  - `CMakeLists.txt` that works standalone **and** as an ament package (it detects
    `ament_cmake`),
  - `package.xml` so colcon discovers it,
  - `library.json` so PlatformIO can consume it through a `symlink://` dependency.
- `shared/CMakeLists.txt` builds all libraries with GoogleTest, coverage and sanitizer options.
- colcon builds with `--base-paths ros2_ws/src shared`.

## Alternatives considered

- **Copy the code into each consumer:** guaranteed drift, especially for the wire format.
- **Git submodule or separate repository:** adds versioning overhead without benefit while one
  person develops everything in one repository.
- **Header-only libraries:** simpler packaging but slower builds and no separate compilation
  units for coverage; the three build descriptions are small enough to maintain.

## Consequences

- One implementation is tested once and used everywhere; the firmware build also compiles every
  shared library with the ESP32 toolchain (GCC 8.4), catching portability problems early.
- Shared libraries must avoid exceptions in their interfaces, dynamic allocation and anything
  beyond C++17 that GCC 8.4 does not support.
- Three small build files per library need to be kept in sync when sources are added.
