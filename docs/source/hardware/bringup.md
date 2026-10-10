# Hardware bring-up

The parameters marked **placeholder** in {doc}`../firmware/parameters` describe how this
particular robot is wired and assembled. The bring-up firmware measures them on the robot.

:::{warning}
Motor tests spin the wheels. Put the robot on a stand with both wheels off the ground, and keep
the Pi link idle while flashing (UART0 is shared, see {doc}`known-issues`).
:::

## 1. Flash and connect

```bash
cd firmware/esp32/robot-firmware
pio run -e bringup -t upload
```

Join the robot's WiFi access point (`robot-debug` unless changed in `secrets.hpp`) and open
`telnet 192.168.4.1 23`. Press `h` and Enter for the command list.

| Command | Action |
|---|---|
| `i` | Initialize both IMUs and print five samples each, in sensor and body axes |
| `a`, `b` | Drive TB6612 channel A or B at 30 % of the maximum duty for 1 s in each direction and report the counts of both encoder connectors |
| `e` | Print both encoder counts for 20 s while you turn a wheel by hand |
| `s` | Stop and disable the motors |

## 2. Motors and encoders → `wiring`

Run `a`, watch which wheel turns, and read the output, for example:

```text
INF channel A positive: J5 +0 counts, J6 +1840 counts
INF channel A negative: J5 +0 counts, J6 -1835 counts
```

| Observation | Parameter |
|---|---|
| Channel A turns the left wheel | `left_motor_channel: 0`, otherwise `1` |
| A positive command turns that wheel backwards | `left_motor_inverted: 1` (same for the right wheel with `b`) |
| The left wheel's counts appear on J6 | `left_encoder_connector: 6`, otherwise `5` |
| The left wheel's counts fall while it turns forwards | `left_encoder_inverted: 1` (same for the right wheel) |

## 3. Encoder resolution → `drivetrain`

Run `e`, mark a wheel and turn it by hand exactly 10 revolutions forwards. Counts per wheel
revolution = total counts / 10, and with ×4 decoding:

```text
counts per wheel revolution = 4 × encoder_pulses_per_motor_rev × gear_ratio
```

If the motor's gear ratio is known from its label or datasheet, solve for the pulses per motor
revolution; otherwise keep the default pulses and adjust `gear_ratio`. Only the product matters
for odometry.

## 4. IMU orientation → `imu_mounting`

Stand the robot upright and run `i`. Each IMU reports which of its axes points up, for example
`'up' is sensor -y`. Then tilt the robot forwards and note which sensor axis now carries part of
gravity. Write the codes so that the body-frame accelerometer reads about `[0 0 +9.8]` when upright
and gains a positive x component when tilted backwards.

| Body axis | Code meaning |
|---|---|
| `*_body_x` (forward) | sensor axis pointing forwards: 1/2/3 = x/y/z, negative if reversed |
| `*_body_y` (left) | sensor axis pointing left |
| `*_body_z` (up) | sensor axis pointing up |

The build rejects combinations that are not a rotation, so a typo cannot slip through.

## 5. Geometry → `geometry`

Measure with a caliper and a scale:

- `wheel_radius`: half the wheel diameter including the tyre.
- `track_width`: distance between the centres of the two tyres' contact patches.
- `body_mass`: the robot without wheels.
- `com_height`: balance the robot on a thin edge across its axle direction, or estimate from the
  CAD model; measure from the wheel axle.

## 6. Apply

Set `measured: true` for every value you determined, then regenerate and rebuild:

```bash
python tools/gen_config.py
cd firmware/esp32/robot-firmware && pio run -e esp32 -t upload
```

The firmware's boot warning about placeholder values disappears once everything is measured.
