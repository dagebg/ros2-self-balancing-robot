# ROS2 Differential Drive Robot

> A personal learning project — building a differential-drive robot from scratch
> using ROS2 on a Raspberry Pi 4, an ESP32 for real-time motor/sensor control,
> and a fully 3D-printed chassis.

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi 4B 4GB | Main computer running ROS2 |
| ESP32 DevKit C V4 | Real-time motor control + IMU bridge |
| 2× N20 6V 100 RPM + Hall encoders | Drive wheels + wheel odometry |
| TB6612FNG dual motor driver | Motor direction + PWM speed control |
| MPU-9250 9-DOF IMU | Accelerometer, gyroscope, magnetometer |
| 2× 7.4V 2000mAh Li-Ion 2S | Main power supply |
| 3D-printed chassis + wheels | Custom mechanical platform |

## Repository layout

```text
ros2-diffdrive-robot/
├── ros2_ws/        ROS2 workspace (packages: diff drive, navigation, etc.)
├── firmware/       ESP32 firmware (motor controller, IMU bridge)
└── docs/           Architecture notes + Sphinx documentation (WIP)
```

## Development setup

- **Dev machines:** Fedora Linux desktop + MacBook (SSH into Pi)
- **Pi ↔ Fedora:** Direct Ethernet connection (SSH + internet sharing)
- **Pi OS:** Ubuntu Server 24.04 + ROS2

## Documentation

- [Architecture overview](docs/ARCHITECTURE.md)
- [Networking setup (Pi ↔ Fedora)](docs/networking-pi-fedora.md)
- Full Sphinx docs: *coming soon*

## Status

🚧 Early WIP — project structure and documentation setup in progress.
