# ADR-0013: Deploy the Pi software as Docker images built in CI

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The Pi needs ROS 2 Jazzy, the robot's packages, their dependencies and the web app, in versions
that match the firmware. Building on the Pi with colcon works, but it is slow and depends on
whatever happens to be installed there. A broken Pi setup should be quick to recreate, and it should
be clear which software version a robot runs.

## Decision

- CI builds a `robot` image (ROS 2 + bridge, model, bringup, rosbridge) and a `web` image (nginx
  with the built web app) and pushes them to GHCR for every push to `main` and every release tag.
- The robot image is built for `linux/arm64` on a native Arm runner; the web image is
  multi-architecture, with the Node build running on the build machine.
- The Pi runs both with `docker compose`, using host networking for ROS 2 discovery and rosbridge.
- Pull requests build both images without pushing, so a Dockerfile change cannot break `main`.

## Alternatives considered

- **colcon build on the Pi:** no registry needed, but builds take long on the Pi and the result
  depends on the Pi's state.
- **Debian packages (bloom):** the standard ROS way for released packages, but heavy for a single
  robot and slow to iterate on.
- **Emulated arm64 builds on x86 runners:** work everywhere, but compiling ROS packages under QEMU
  takes tens of minutes.

## Consequences

- Reinstalling the Pi means: UART setup, Docker, `docker compose up`.
- Image tags (`X.Y.Z`, `sha-…`) tie the running software to a commit.
- The robot image is about 2 GB because it is based on the full `ros:jazzy` image; a slimmer
  runtime image is possible later.
- GHCR packages are private by default; they have to be made public once (or the Pi has to log in)
  before `docker compose pull` works.
