# ADR-0005: Use GoogleTest for all C++ tests

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

C++ code is tested in three build systems: PlatformIO (firmware), CMake (shared libraries) and
colcon/ament (ROS 2 packages). PlatformIO defaults to Unity, a C framework; ROS 2 uses GoogleTest
through `ament_cmake_gtest`.

## Decision

Use GoogleTest everywhere: PlatformIO's `test_framework = googletest` for firmware host tests,
`FetchContent` GoogleTest for the shared libraries' standalone CMake build, and
`ament_cmake_gtest` inside ROS 2.

## Alternatives considered

- **Unity for firmware, GoogleTest for ROS 2:** two assertion styles and two mental models, and
  Unity's C API fits C++ code poorly (no fixtures, no typed or parameterized tests).
- **Catch2 or doctest:** fine frameworks, but ROS 2 tooling and most C++ job postings assume
  GoogleTest.

## Consequences

- Tests read the same in every part of the repository, and test helpers can move between them.
- Firmware tests run on the host only; on-target tests are not planned because hardware behavior
  is covered by the bring-up firmware.
