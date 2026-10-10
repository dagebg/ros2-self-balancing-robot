"""Simulated robot: Gazebo + virtual ESP32 + the unchanged bringup (bridge, model, rosbridge).

    ros2 launch robot_sim sim.launch.py              # headless
    ros2 launch robot_sim sim.launch.py gui:=true    # with the Gazebo window

Everything above the serial line is the same as on the robot, so the web app and RViz work
against the simulation without changes.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup import robot_params

UART_LINK = "/tmp/robot_sim_uart"


def simulation(context):
    sim_share = Path(get_package_share_directory("robot_sim"))
    description_share = Path(get_package_share_directory("robot_description"))
    params = robot_params.load()
    wheel_radius = float(robot_params.value(params, "geometry", "wheel_radius"))

    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    world = str(sim_share / "worlds" / "balance.sdf")
    gz_args = f"-r -v 1 {world}" if gui else f"-r -s -v 1 {world}"

    model = xacro.process_file(
        str(description_share / "urdf" / "robot.urdf.xacro"), mappings={"sim": "true"}
    ).toxml()

    bridge_geometry = robot_params.bridge_parameters(params)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "robot", "-string", model, "-z", str(wheel_radius + 0.001)],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": str(sim_share / "config" / "gz_bridge.yaml")}],
        ),
        Node(
            package="robot_sim",
            executable="virtual_esp32",
            output="screen",
            parameters=[
                {
                    "uart_link": UART_LINK,
                    "wheel_radius": bridge_geometry["wheel_radius"],
                    "track_width": bridge_geometry["track_width"],
                    "counts_per_wheel_rev": bridge_geometry["counts_per_wheel_rev"],
                    "auto_arm": LaunchConfiguration("auto_arm").perform(context) == "true",
                }
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("robot_bringup"))
                    / "launch"
                    / "robot.launch.py"
                )
            ),
            launch_arguments={
                "port": UART_LINK,
                "rosbridge": LaunchConfiguration("rosbridge"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false", description="Gazebo window"),
            DeclareLaunchArgument("rosbridge", default_value="true", description="Web app API"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "auto_arm",
                default_value="true",
                description="Arm as soon as calibrated, before the spawned robot can tip over",
            ),
            OpaqueFunction(function=simulation),
        ]
    )
