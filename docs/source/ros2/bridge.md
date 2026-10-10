# robot_bridge

`robot_bridge` connects the ESP32's UART protocol (see {doc}`../protocol`) to ROS 2. It is a
composable node (`robot_bridge::BridgeNode`) and a standalone executable (`bridge_node`).

```bash
ros2 run robot_bridge bridge_node --ros-args \
  -p wheel_radius:=0.0325 -p track_width:=0.15 -p counts_per_wheel_rev:=4200.0
```

Normally it is started by `robot_bringup`'s launch file, which takes the geometry from
`config/robot_params.yaml`; started on its own, it refuses to run without these parameters.

## Interfaces

| Name | Type | Direction | Notes |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | in | `linear.x` and `angular.z` are forwarded |
| `balance/state` | `robot_interfaces/BalanceState` | out | controller state, faults, attitude, motor commands, loop timing |
| `odom` | `nav_msgs/Odometry` | out | wheel odometry with pitch compensation |
| TF `odom` → `base_footprint` | `tf2_msgs/TFMessage` | out | when `publish_tf` is true |
| `joint_states` | `sensor_msgs/JointState` | out | `body_pitch_joint` (measured pitch) and the wheel angles and rates relative to the chassis |
| `imu/data` | `sensor_msgs/Imu` | out | MPU-9250, calibrated body axes; orientation from the ESP32 (yaw integrated from the gyro) |
| `imu/secondary` | `sensor_msgs/Imu` | out | MPU-6050, calibrated body axes, no orientation |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | `link` and `controller` status |
| `/rosout` | log | out | firmware log lines, prefixed with `[esp32]` |
| `~/arm` | `std_srvs/SetBool` | service | `true` arms, `false` disarms; answers after the ESP32's Ack |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/serial0` | serial device |
| `baud` | 460800 | must match the firmware |
| `wheel_radius`, `track_width` | required | [m], from `robot_params.yaml` |
| `counts_per_wheel_rev` | required | encoder counts per wheel revolution (×4 decoding) |
| `odom_frame`, `base_frame`, `imu_frame` | `odom`, `base_footprint`, `imu_link` | frame ids |
| `pitch_joint`, `left_wheel_joint`, `right_wheel_joint` | `body_pitch_joint`, `left_wheel_joint`, `right_wheel_joint` | joint names in `joint_states` |
| `publish_tf` | true | broadcast `odom` → `base_footprint` |
| `command_rate` | 20.0 | [Hz] rate at which velocity commands are sent |
| `cmd_vel_timeout` | 0.5 | [s] older commands are replaced by zero velocity |
| `reconnect_period` | 1.0 | [s] retry interval while the port cannot be opened |
| `ack_timeout` | 1.0 | [s] how long `~/arm` waits for the ESP32 |
| `gains.{pitch,velocity,yaw_rate}.{kp,ki,kd}` | firmware defaults | live PID gains |

## Behaviour

**Commands and heartbeat.** The bridge sends `CmdVel` at `command_rate` whether or not new
`cmd_vel` messages arrive. If the last `cmd_vel` is older than `cmd_vel_timeout`, it sends zero
velocity. The stream doubles as the heartbeat for the ESP32's 500 ms link supervision, so if the
bridge dies, the robot stops (and keeps balancing).

**Gains.** The gain parameters default to the values compiled into the firmware
(`balance_core::BalanceConfig`, read from the same header). Changing one, for example

```bash
ros2 param set /robot_bridge gains.pitch.kp 30.0
```

validates it (finite, non-negative) and sends `SetGains` for that loop. After every (re)connection
the bridge pushes all gains once telemetry arrives, so a rebooted ESP32 runs the gains ROS shows.

**Arming.** `ros2 service call /robot_bridge/arm std_srvs/srv/SetBool "{data: true}"` sends
`SetArmed` and answers with the ESP32's acknowledgement: success, *rejected* (fault latched or gyro
not yet calibrated) or *timeout*.

**Reconnection.** If the port cannot be opened or fails (for example after unplugging), the
bridge closes it, reports the link as an error in `diagnostics` and retries every
`reconnect_period`.

## Tests

| Test | What it covers |
|---|---|
| `test_conversions` | quaternion maths, telemetry → `BalanceState`, IMU messages, odometry incl. pitch compensation and timestamp wrap |
| `test_serial_port` | the termios wrapper and framing over a real pseudo-terminal |
| `test_bridge_node` | the running node against a fake ESP32 on a pseudo-terminal: telemetry → topics, `cmd_vel` forwarding and timeout, arming (ok, rejected, timeout), live gains, gain sync on connect |
