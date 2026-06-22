# Mechanical Hardware

This document tracks the mechanical design of the robot chassis and related 3D‑printed parts that I design and print myself, primarily using Autodesk Fusion 360 and my BambuLab P1S printer.

---

## Scope

The mechanical documentation in this file covers: 

- Overall chassis concept and footprint  
- Wheel, caster, and support geometry  
- Mounting points for electronics and sensors (mechanical aspects only)  
- 3D‑printing considerations and tolerances  
- Version history of my CAD models and printed prototypes  

---

## Third-Party CAD Models

The following CAD models were sourced from GrabCAD for reference and assembly visualisation. They are **not** designed by me. All rights remain with the original authors.

| File | Description | Author / Source |
|------|-------------|-----------------|
| `N20_motor_encoder.stl` | N20 micro gear motor with magnetic encoder | GrabCAD Library — [N20 Motor with Magnetic Encoder](https://grabcad.com/library/n20-motor-with-magnetic-encoder-1) |
| `Raspberry Pi 4 Model B.STEP` | Raspberry Pi 4 Model B single-board computer | GrabCAD Library — [Raspberry Pi 4 Model B](https://grabcad.com/library/raspberry-pi-4-model-b-1) |
| `ESP32.STEP` | ESP32 DevKit C V4 development board | GrabCAD Library — [ESP32-DevKitC](https://grabcad.com/library/esp-32-1) |
| `MPU-9250.stp` | MPU-9250 9-DOF IMU breakout board | GrabCAD Library — [MPU-9250 Module](https://grabcad.com/library/mpu-9250-1) |
| `TB6612FNG_motor_driver.step` | TB6612FNG dual motor driver breakout board | GrabCAD Library — [TB6612FNG Motor Driver](https://grabcad.com/library/tb6612fng-1) |
| `Battery_Li-ion.stp` | 7.4 V 2S Li‑Ion battery pack (2000 mAh) | GrabCAD Library — [Li-Ion Battery Pack](https://grabcad.com/library/li-ion-battery-7-4v-1) |
| `Pi_cooler.step` | Heatsink / cooling assembly for Raspberry Pi 4 | GrabCAD Library — [Raspberry Pi 4 Cooler](https://grabcad.com/library/raspberry-pi-4-cooler-1) |

> **Note:** Third-party models in this folder are used solely for mechanical reference and assembly planning. They are not original designs of this project.
