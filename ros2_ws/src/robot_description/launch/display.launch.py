"""Show the robot model in RViz, with sliders for its joints (no hardware needed)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("robot_description")
    description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([share, "urdf", "robot.urdf.xacro"])]),
        value_type=str,
    )
    gui = LaunchConfiguration("gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Joint sliders"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([share, "rviz", "robot.rviz"])],
            ),
        ]
    )
