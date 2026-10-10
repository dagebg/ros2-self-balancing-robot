# ADR-0001: Record architecture decisions

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The project spans firmware, ROS 2, a web app, simulation and CI. Many choices (language, protocol,
libraries) are hard to reverse and are not obvious from the code alone. Months later, neither the
author nor a reviewer can tell why something was done unless it is written down.

## Decision

Record every significant decision as a short Markdown ADR in `docs/source/development/adr/`, using
the template in `0000-template.md`. ADRs are part of the Sphinx documentation and are reviewed in
the same pull request as the change they describe.

## Alternatives considered

- **Commit messages only:** too scattered to read as a whole, and they cannot be updated.
- **A wiki:** lives outside the repository and drifts from the code.

## Consequences

- Decisions can be reviewed in pull requests and are versioned with the code.
- A small amount of extra writing per significant change.
