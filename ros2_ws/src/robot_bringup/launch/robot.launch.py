"""Brings up the robot: model, ESP32 bridge, and optionally rosbridge, joystick and RViz.

ros2 launch robot_bringup robot.launch.py
ros2 launch robot_bringup robot.launch.py port:=/dev/ttyAMA0 joystick:=true rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from robot_bringup import robot_params


def bridge(context):
    params = robot_params.load()
    actions = [
        Node(
            package="robot_bridge",
            executable="bridge_node",
            name="robot_bridge",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("robot_bridge"), "config", "bridge.yaml"]),
                robot_params.bridge_parameters(params),
                {"port": LaunchConfiguration("port").perform(context)},
            ],
        )
    ]
    unmeasured = robot_params.placeholders(params)
    if unmeasured:
        actions.append(
            LogInfo(msg=f"robot_params.yaml placeholders (not measured): {', '.join(unmeasured)}")
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]
                ),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/serial0", description="ESP32 UART"),
            DeclareLaunchArgument("rosbridge", default_value="true", description="Web app API"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("joystick", default_value="false", description="Gamepad"),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
            ),
            OpaqueFunction(function=bridge),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                parameters=[{"port": LaunchConfiguration("rosbridge_port")}],
                condition=IfCondition(LaunchConfiguration("rosbridge")),
            ),
            Node(
                package="joy",
                executable="joy_node",
                condition=IfCondition(LaunchConfiguration("joystick")),
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("robot_bringup"), "config", "joystick.yaml"]
                    )
                ],
                condition=IfCondition(LaunchConfiguration("joystick")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("robot_description"), "rviz", "robot.rviz"]
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
