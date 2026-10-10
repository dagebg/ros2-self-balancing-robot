# Bill of materials

| Qty | Component | Role |
|---|---|---|
| 1 | Raspberry Pi 4B, 4 GB | Main computer running ROS 2 |
| 1 | ESP32 DevKitC V4 (AZ-Delivery, ESP32-WROOM-32) | Real-time motor control and sensor hub |
| 2 | N20 gear motor, 100 RPM, with Hall encoder | Drive wheels and wheel odometry |
| 1 | TB6612FNG dual H-bridge breakout | Motor direction and PWM speed control |
| 1 | MPU-9250 9-DOF IMU breakout | Primary IMU: accelerometer, gyroscope, magnetometer |
| 1 | MPU-6050 6-DOF IMU breakout | Secondary IMU: accelerometer, gyroscope |
| 2 | 7.4 V 2000 mAh Li-Ion 2S pack | Main power source |
| 3 | HW-140 DC-DC converter | 5.1 V (Pi), 5 V (ESP32), 11.5 V (motors) rails |
| 1 | 2-way DIP switch | Main power switch |
| — | 3D-printed chassis, decks and wheels | Mechanical platform |

## IMU placement

The two IMUs are mounted at different locations and orientations on purpose, to experiment with
sensor fusion.

| IMU | Location | Orientation | I2C address | Function |
|---|---|---|---|---|
| MPU-9250 | Lower deck | Facing forward | `0x69` (AD0 high) | Primary IMU |
| MPU-6050 | Second deck | Facing backward | `0x68` (AD0 low) | Secondary / reference IMU |

```{image} ../../img/mpu9250_lower_deck_front.JPG
:width: 48%
:alt: MPU-9250 on the lower deck
```
```{image} ../../img/mpu6050_second_deck_back.JPG
:width: 48%
:alt: MPU-6050 on the second deck
```
