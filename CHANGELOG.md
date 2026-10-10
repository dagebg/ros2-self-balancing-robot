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

### Changed

- CI restructured: a single `lint` job runs pre-commit, all tool versions are pinned and cached,
  and superseded runs are cancelled.
- `.clang-format` moved to the repository root so one C++ style applies everywhere.

### Fixed

- Documentation build warnings, broken links and leftover citation markers.
- Documentation is now built on every pull request, with warnings treated as errors.
