# Web app

A browser control panel in `webapp/` (React, TypeScript, Vite). It talks to the robot through
`rosbridge` on the Pi, which the bringup starts by default (see
{doc}`development/adr/0012-webapp-over-rosbridge`).

| Panel | Function |
|---|---|
| Status | connection, controller state, latched faults, pitch, motor commands, loop time, odometry |
| Arming | Arm / Disarm through `/robot_bridge/arm`, with the ESP32's answer |
| Drive | virtual joystick (drag) and keyboard (arrows / WASD); enabled only while balancing |
| Controller gains | live PID tuning through the bridge's parameter services |
| Telemetry | the last 10 s of pitch and both motor commands |

## Running

```bash
cd webapp
npm install
npm run dev            # http://localhost:5173
```

By default the app connects to `ws://<page host>:9090`; point it elsewhere with a query parameter,
for example `http://localhost:5173/?ros=ws://10.42.0.100:9090` for the robot on the development
network, or `?ros=ws://localhost:9090` for the simulation:

```bash
ROS_DOCKER_ARGS="-p 9090:9090" tools/ros-docker.sh \
  "cd ros2_ws && source install/setup.bash && ros2 launch robot_sim sim.launch.py"
```

`npm run build` writes a static site to `webapp/dist` that any web server can host.

## How it talks to ROS

| UI action | ROS interface | Type |
|---|---|---|
| status, telemetry | `/balance/state` (throttled to 25 Hz) | `robot_interfaces/msg/BalanceState` |
| position | `/odom` (throttled to 10 Hz) | `nav_msgs/msg/Odometry` |
| drive | `/cmd_vel`, re-sent every 100 ms while active, one stop on release | `geometry_msgs/msg/Twist` |
| arm / disarm | `/robot_bridge/arm` | `std_srvs/srv/SetBool` |
| read gains | `/robot_bridge/get_parameters` | `rcl_interfaces/srv/GetParameters` |
| change a gain | `/robot_bridge/set_parameters` | `rcl_interfaces/srv/SetParameters` |

No custom server-side code is needed: rosbridge exposes these standard interfaces. If the browser
tab closes while driving, the bridge's `cmd_vel` timeout stops the robot within 0.5 s.

## Structure and tests

```text
webapp/src/
├── lib/            framework-free logic: robot types, input mapping, gains, plot buffer, URL
│   ├── client.ts   RobotClient interface + RosbridgeClient (the only code that touches roslib)
│   └── useRobot.ts React hook that connects and exposes the latest state
├── components/     StatusPanel, ArmControls, DrivePad, GainsPanel, TelemetryChart
└── test/           FakeClient (an in-memory RobotClient) and test setup
```

Components depend only on the `RobotClient` interface, so they are tested against `FakeClient`:

| Command | What runs |
|---|---|
| `npm test` | unit tests of the logic modules and component tests of the whole app (Vitest, Testing Library) |
| `npm run lint`, `npm run typecheck` | ESLint, Prettier, TypeScript |
| `ROSBRIDGE_URL=ws://localhost:9090 npm run test:integration` | `RosbridgeClient` against a running simulation or robot: state, arming, gain read/write and rejection, driving |

CI runs everything except the integration test, which needs a running simulation.
