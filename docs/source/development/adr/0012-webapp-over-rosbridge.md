# ADR-0012: Serve the web app from the Pi and talk to ROS through rosbridge

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The project goal includes controlling the robot from a browser: drive, arm, watch telemetry and
tune the controller. The ESP32 could host a page itself over its WiFi access point, or the Pi could
serve one that talks to ROS 2.

## Decision

- The web app is a static single-page application (React, TypeScript, Vite) served from the Pi
  side.
- It talks to ROS 2 through `rosbridge_server` (WebSocket, JSON) using `roslib`, and only through
  standard interfaces: topics, the bridge's `~/arm` service and the node's parameter services.
- All browser-side code depends on a small `RobotClient` interface; `RosbridgeClient` is its only
  implementation that touches the network.

## Alternatives considered

- **Web page on the ESP32:** works without the Pi, but adds WiFi and HTTP load to the real-time
  controller, duplicates the command path, and bypasses ROS (no logging, no simulation).
- **Custom backend (e.g. FastAPI with rclpy):** more control over the API, but one more service to
  write, test and deploy, for what rosbridge already provides.
- **Foxglove Studio:** excellent for visualization and debugging, but not a purpose-built
  control panel, and the project wants its own UI as part of the work.

## Consequences

- The same app drives the real robot and the simulation, and a dropped browser is covered by the
  bridge's `cmd_vel` timeout.
- rosbridge has no authentication: the API must only be reachable on a trusted network (the
  development link or the robot's own access point).
- Message type names in the client must match the ROS interfaces; an opt-in integration test checks
  the client against a running simulation.
