# Architecture

## Components

- Dev PCs: Fedora Linux desktop and MacBook (used for development and SSH)
- Robot: Raspberry Pi 4 Model B 4GB running Ubuntu Server 24.04 and ROS2
- Microcontroller: ESP32 for motor and sensor control
- Hardware: TB6612FNG motor driver, 2× N20 motors with encoders, MPU-9250 IMU, Li-Ion batteries
- Mechanics: 3D-printed robot chassis, wheels, and support hardware

## Connections

- Fedora PC ↔ Raspberry Pi: direct Ethernet (SSH, internet sharing) for development  
  (see `docs/networking-pi-fedora.md` for details)

## ROS2 (planned)

- The Raspberry Pi runs ROS2 nodes that:
  - Subscribe to `/cmd_vel` for velocity commands
  - Publish odometry and IMU data
- The ESP32 handles:
  - Motor PWM output
  - Reading wheel encoders and IMU data
  - Sending this data back to the Raspberry Pi

This document will be updated as the system evolves.