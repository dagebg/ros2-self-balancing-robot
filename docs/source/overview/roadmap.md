# Roadmap

The project is built in small, reviewable milestones. Each one lands with its code, tests,
CI job and documentation together.

| # | Milestone | Status |
|---|---|---|
| 1 | Documentation foundation: Sphinx, MyST, Doxygen, ADRs, hardware reference | Done |
| 2 | CI foundation: pre-commit, formatting, static analysis | Done |
| 3 | Firmware refactor to modern C++17, host unit-test harness | Done |
| 4 | Binary UART protocol shared by firmware and ROS 2 | Done |
| 5 | Firmware drivers: IMUs, motor driver, encoders, hardware bring-up tool | Planned |
| 6 | Balance core: sensor fusion, cascaded PID, odometry, safety state machine | Planned |
| 7 | Firmware application: FreeRTOS tasks, comms, live tuning | Planned |
| 8 | ROS 2 bridge node | Planned |
| 9 | Robot description (URDF) and bringup launch files | Planned |
| 10 | Gazebo software-in-the-loop simulation | Planned |
| 11 | Web app control panel | Planned |
| 12 | Release and deployment pipeline (firmware images, Docker on the Pi) | Planned |
| 13 | Showcase README and v0.1.0 release | Planned |

## Future work

- Battery-voltage sensing (needs a voltage divider to a free ADC1 pin) and a low-battery cut-off.
- State-space (LQR) balance controller as an alternative to cascaded PID.
- Magnetometer calibration and yaw fusion from the MPU-9250.
