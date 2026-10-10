# robot_bringup

One launch file starts everything the robot needs on the Pi:

```bash
ros2 launch robot_bringup robot.launch.py
```

| Started | Condition |
|---|---|
| `robot_state_publisher` with the URDF | always |
| `robot_bridge`, geometry from `robot_params.yaml` | always |
| `rosbridge_websocket` on port 9090 (web app API) | `rosbridge:=true` (default) |
| `joy_node` + `teleop_twist_joy` | `joystick:=true` |
| RViz with the robot view | `rviz:=true` |

| Argument | Default | Meaning |
|---|---|---|
| `port` | `/dev/serial0` | ESP32 UART device |
| `rosbridge` | `true` | start the WebSocket API for the web app |
| `rosbridge_port` | `9090` | its port |
| `joystick` | `false` | gamepad teleoperation |
| `rviz` | `false` | open RViz (needs a display) |

At start-up the launch file logs every parameter in `robot_params.yaml` that is still a placeholder.

## Driving

- **Gamepad:** `joystick:=true`; hold the left shoulder button and steer with the left stick
  (mapping in `config/joystick.yaml`, speeds capped at 0.15 m/s and 1.2 rad/s).
- **Keyboard:** install `ros-jazzy-teleop-twist-keyboard` and run
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in a terminal.
- **Browser:** the web app, through rosbridge.

Arm first: `ros2 service call /robot_bridge/arm std_srvs/srv/SetBool "{data: true}"`, then hold the
robot upright until it starts balancing.

## Parameters from robot_params.yaml

`robot_bringup.robot_params` reads the installed `robot_params.yaml` and derives what the bridge
needs, for example

$$\text{counts per wheel revolution} = 4 \times \text{pulses per motor revolution} \times \text{gear ratio}$$

so no constant is typed into a launch file.

## Tests

- `test_robot_params.py`: the derivations, and that the installed file is the repository's.
- `test_robot_launch.py` (launch_testing): starts the real launch file **without hardware** and
  checks that the model is published, that the bridge keeps running and reports the missing ESP32
  in `/diagnostics`, that it received its geometry from `robot_params.yaml`, and that every process
  exits cleanly.
