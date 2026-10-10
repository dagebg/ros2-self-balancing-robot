# Mechanical Hardware

This document tracks the mechanical design of the robot chassis and related 3D‑printed parts that I design and print myself, primarily using Autodesk Fusion 360 and Siemens NX and my BambuLab P1S printer.

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
| `ESP32.STEP` | ESP32 DevKit C V4 development board | GrabCAD Library — [ESP32 DevKitC V4](https://grabcad.com/library/esp32-devkitc-v4-1) |
| `MPU-9250.stp` | MPU-9250 9-DOF IMU breakout board | GrabCAD Library — [MPU-9250](https://grabcad.com/library/mpu-9250-1) |
| `TB6612FNG_motor_driver.step` | TB6612FNG dual motor driver breakout board | GrabCAD Library — [TB6612FNG](https://grabcad.com/library/tb6612fng-2) |
| `Battery_Li-ion.stp` (not yet in repository) | 7.4 V 2S 18650 Li‑Ion battery pack (2000 mAh) | GrabCAD Library — [2S 7.4V 2000mAh 18650 Battery Pack](https://grabcad.com/library/2s-7-4v-2000mah-18650-battery-pack-1) |
| `Pi_cooler.step` (not yet in repository) | Heatsink / cooling assembly for Raspberry Pi 4 | GrabCAD Library — [Raspberry Pi 4 Cooler](https://grabcad.com/library/raspberry-pi-4-cooler-1) |
| HW-140 Buck Converter | No suitable CAD model found — placeholder geometry to be modelled manually if needed for assembly ||

> **Note:** Third-party models in this folder are used solely for mechanical reference and assembly planning. They are not original designs of this project.
