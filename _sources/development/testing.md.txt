# Testing strategy

A balancing robot that falls over is a poor debugging tool. Most behavior is therefore verified
off the robot, and the hardware is the last step, not the first.

```mermaid
flowchart BT
  U["Unit tests<br/>host, milliseconds"] --> I["Integration tests<br/>host + containers, seconds"]
  I --> S["Simulation (SIL)<br/>Gazebo, headless in CI"]
  S --> H["Hardware bring-up<br/>on the robot, manual"]
```

## Layers

| Layer | What it covers | Where it runs |
|---|---|---|
| Unit | Protocol codec, filters, controllers, kinematics, drivers against fake buses | Host (macOS/Linux) and CI |
| Integration | ROS 2 bridge against a fake ESP32 on a pseudo-terminal; launch files | `ros:jazzy` container and CI |
| Simulation | The real balance controller code keeping a simulated robot upright | Gazebo Harmonic, headless, CI |
| Hardware | IMU orientation, motor direction, encoder counts, final tuning | Robot, with the bring-up firmware |

## Principles

- **Hardware behind interfaces.** Firmware logic talks to small interfaces (I2C bus, PWM output,
  counter, clock) rather than to Arduino directly. Tests supply fakes that record and replay bus
  traffic.
- **One implementation, tested once, used everywhere.** The protocol codec and the balance core
  are plain C++17 libraries shared by the firmware, the ROS 2 bridge and the simulator.
- **Closed-loop tests.** Controllers are tested against a plant model (an inverted pendulum on
  wheels), not just for individual outputs, so a test fails if the robot would fall.
- **Same tools locally and in CI.** Every CI job is reproducible with the commands in
  {doc}`setup`.

## Frameworks

GoogleTest is used for all C++ tests: PlatformIO's native test runner for firmware, CMake/CTest
for the shared libraries, and `ament_cmake_gtest` for ROS 2 packages. Python launch tests use
`launch_testing`, and the web app uses Vitest.
