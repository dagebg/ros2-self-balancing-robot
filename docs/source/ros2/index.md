# ROS 2

The Raspberry Pi runs ROS 2 Jazzy. The workspace in `ros2_ws/src` builds together with the shared
C++ libraries:

| Package | Type | Purpose |
|---|---|---|
| `robot_interfaces` | messages | `BalanceState` |
| `robot_bridge` | C++ node / component | ESP32 link ↔ ROS 2 graph |
| `robot_description` | URDF / xacro | robot model from `robot_params.yaml`, RViz preview |
| `robot_bringup` | launch / Python | starts model, bridge, rosbridge and teleoperation |
| `robot_protocol` | C++ library (`shared/`) | protocol codec, also used by the firmware |
| `balance_core` | C++ library (`shared/`) | odometry and controller defaults, also used by the firmware |

:::{toctree}
:maxdepth: 1

bringup
bridge
description
:::

## Building

On a development machine, everything runs in the `ros:jazzy` container (see
{doc}`../development/setup`):

```bash
tools/ros-docker.sh colcon-build
tools/ros-docker.sh colcon-test
```

On the Pi (Ubuntu 24.04 with ROS 2 Jazzy installed):

```bash
cd ros2_ws
rosdep install --from-paths src ../shared --ignore-src -y
colcon build --base-paths src ../shared/*/
source install/setup.bash
```

`shared/` contains one directory per library plus a `CMakeLists.txt` for standalone tests; colcon
is pointed at the library directories (`../shared/*/`) so that file is not mistaken for a package.
