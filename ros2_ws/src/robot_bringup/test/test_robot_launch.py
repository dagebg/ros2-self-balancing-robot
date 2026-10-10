"""Starts the real bringup without hardware and checks the pieces come up and talk.

No ESP32 is connected: the bridge must keep running, retry the port and report the missing link
in /diagnostics, while robot_state_publisher serves the model.
"""

import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_bringup import robot_params
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robot_bringup"), "launch", "robot.launch.py"])
        ),
        launch_arguments={"port": "/dev/no-esp32-here", "rosbridge": "false"}.items(),
    )
    return LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestRobotLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("launch_tester")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition():
                return True
        return False

    def test_robot_description_is_published(self):
        received = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            String, "robot_description", lambda msg: received.append(msg.data), qos
        )
        self.assertTrue(self.spin_until(lambda: received))
        self.assertIn("left_wheel_joint", received[0])
        self.assertIn("body_pitch_joint", received[0])

    def test_bridge_reports_missing_link(self):
        link_errors = []

        def on_diagnostics(msg):
            for status in msg.status:
                if status.name.endswith("link") and status.level == DiagnosticStatus.ERROR:
                    link_errors.append(status)

        self.node.create_subscription(DiagnosticArray, "diagnostics", on_diagnostics, 10)
        self.assertTrue(self.spin_until(lambda: link_errors))

    def test_bridge_uses_geometry_from_robot_params(self):
        client = self.node.create_client(GetParameters, "robot_bridge/get_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=20.0))
        future = client.call_async(
            GetParameters.Request(names=["wheel_radius", "counts_per_wheel_rev"])
        )
        self.assertTrue(self.spin_until(future.done))
        values = [v.double_value for v in future.result().values]
        expected = robot_params.bridge_parameters(robot_params.load())
        self.assertAlmostEqual(values[0], expected["wheel_radius"])
        self.assertAlmostEqual(values[1], expected["counts_per_wheel_rev"])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
