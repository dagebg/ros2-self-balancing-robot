# ADR-0011: Simulate with a virtual ESP32 running the firmware code behind the real bridge

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

A simulation is only as useful as the amount of real code it exercises. The control code is
already portable (`balance_core`), but so is everything else the firmware does above the hardware:
bias calibration, fusion, safety, protocol handling and link supervision (`ControlLoop`,
`CommsService`). On the Pi side, `robot_bridge` and the bringup launch files are what the web app
and RViz talk to.

## Decision

The simulation replaces only the hardware:

- Gazebo Harmonic provides physics, an IMU and wheel joint states, and takes wheel torques.
- `virtual_esp32`, a ROS node, compiles the firmware's portable `app`, `drivers` (headers),
  `hal` (interfaces) and `logging` sources for the host. It feeds Gazebo data into `ControlLoop`,
  converts motor commands to torques with the same DC-motor model as the plant model, and serves
  `CommsService` over a pseudo-terminal.
- The unchanged `robot_bridge` opens that pseudo-terminal like a serial port; the unchanged bringup
  starts the model and rosbridge.

## Alternatives considered

- **A ROS controller node using `balance_core` directly:** simpler, but bypasses calibration,
  safety gating, the protocol and the bridge, so a large part of the system would never run in
  simulation.
- **`gz_ros2_control` with a controller plugin:** the standard ROS approach, but the robot's
  controller does not live in ROS; it would mean a second, simulation-only control path.
- **Emulating the ESP32 (QEMU):** maximal fidelity, but slow, fragile and far more effort than
  the portable-code boundary already allows.

## Consequences

- The firmware's control and communication code, the protocol, the bridge and the launch files are
  tested together in CI on every pull request.
- The web app and RViz can be developed against the simulation with no changes.
- Hardware-specific code (drivers' register access, PCNT, LEDC, FreeRTOS tasks) is still only
  exercised by its unit tests and on the robot.
- `robot_sim`'s build reaches into `firmware/` by relative path, coupling it to the repository
  layout.
