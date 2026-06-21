# ros2-diffdrive-robot

> A personal learning project — building a differential-drive robot from scratch using ROS2, including 3D printing housing etc.

## Project structure

```text
ros2-diffdrive-robot/
├── ARCHITECTURE.md           # High-level system overview
├── README.md                 # This file
├── docs/
│   ├── hardware-software.md  # Hardware and dev environment (WIP)
│   └── networking-pi-fedora.md
└── source/                   # ROS2 workspace src/ (to be used later)
```

High-level architecture (logical):

```text
[Fedora Dev PC]  <--Ethernet-->  [Raspberry Pi 4 (ROS2)]
                                       |
                                       | USB / serial
                                       v
                                  [ESP32 MCU]
                                       |
                      +----------------+----------------+
                      |                                 |
                 [Motors + encoders]              [MPU-9250 IMU]
```

## Documentation

- [Architecture overview](ARCHITECTURE.md)
- [Raspberry Pi ↔ Fedora Ethernet setup](docs/networking-pi-fedora.md)

## Status

Early WIP. 
