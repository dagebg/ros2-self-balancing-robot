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

### Changed

- CI restructured: a single `lint` job runs pre-commit, all tool versions are pinned and cached,
  and superseded runs are cancelled.
- `.clang-format` moved to the repository root so one C++ style applies everywhere.
- Firmware converted to modern C++17 (`.hpp`, namespaces, `constexpr` configuration, no Arduino
  `String`); `WifiDebug` replaced by `robot::telemetry::TelnetSink`; platform pinned to
  `espressif32@7.1.3`. clang-tidy now includes the C++ Core Guidelines checks.
- Firmware logs no longer go to `Serial`, which is reserved for the Raspberry Pi link.

### Fixed

- Documentation build warnings, broken links and leftover citation markers.
- Documentation is now built on every pull request, with warnings treated as errors.
- Firmware failed to build from a fresh clone: `SECRETS.h.example` defined different macro names
  than `main.cpp` used. Credentials now fall back to `secrets.example.hpp` automatically.
