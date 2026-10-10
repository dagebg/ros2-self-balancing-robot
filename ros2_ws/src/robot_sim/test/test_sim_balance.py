"""End-to-end simulation test: Gazebo, the virtual ESP32 (firmware code), the serial protocol over
a pseudo-terminal, the real bridge and bringup. Checks arming through the bridge's service, lets
the robot balance, drives it forward and checks Gazebo's ground truth.
"""

import math
import time
import unittest

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from robot_interfaces.msg import BalanceState
from std_srvs.srv import SetBool

ARMED = BalanceState.STATE_ARMED
BALANCING = BalanceState.STATE_BALANCING


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("robot_sim"), "launch", "sim.launch.py"])
        ),
        launch_arguments={"gui": "false", "rosbridge": "false"}.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestSimulatedBalancing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sim_tester")
        cls.state = None
        cls.states = []
        cls.ground_truth = None
        cls.node.create_subscription(BalanceState, "balance/state", cls.on_state, 10)
        cls.node.create_subscription(Odometry, "sim/ground_truth", cls.on_truth, 10)
        cls.cmd_vel = cls.node.create_publisher(Twist, "cmd_vel", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_state(cls, msg):
        cls.state = msg
        cls.states.append(msg)

    @classmethod
    def on_truth(cls, msg):
        cls.ground_truth = msg

    def spin_for(self, seconds, every=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if every:
                every()

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return True
        return False

    def test_balances_and_drives_through_the_full_stack(self):
        # 1. Telemetry flows: Gazebo -> virtual ESP32 -> pseudo-terminal -> bridge -> ROS.
        self.assertTrue(self.spin_until(lambda: self.state is not None, 60.0), "no telemetry")

        # 2. The virtual ESP32 calibrates its gyro and arms itself (auto_arm), as a hand holding
        #    the real robot would. Arming through the bridge's service is then acknowledged by the
        #    firmware code (arming is idempotent).
        self.assertTrue(
            self.spin_until(lambda: self.state.state in (ARMED, BALANCING), 30.0), "never armed"
        )
        client = self.node.create_client(SetBool, "robot_bridge/arm")
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        future = client.call_async(SetBool.Request(data=True))
        self.assertTrue(self.spin_until(future.done, 10.0))
        self.assertTrue(future.result().success, future.result().message)

        # 3. It starts balancing and stays up.
        self.assertTrue(
            self.spin_until(lambda: self.state.state == BALANCING, 10.0), "did not start"
        )
        self.states.clear()
        self.spin_for(8.0)
        self.assertTrue(all(s.state == BALANCING for s in self.states), "fell over")
        max_pitch = max(abs(s.pitch) for s in self.states)
        self.assertLess(max_pitch, math.radians(10.0))

        # 4. Drive forwards; Gazebo's ground truth must show the motion.
        self.assertTrue(self.spin_until(lambda: self.ground_truth is not None, 10.0))
        start_x = self.ground_truth.pose.pose.position.x
        twist = Twist()
        twist.linear.x = 0.1
        self.states.clear()
        self.spin_for(6.0, every=lambda: self.cmd_vel.publish(twist))
        travelled = self.ground_truth.pose.pose.position.x - start_x
        self.assertGreater(travelled, 0.25)
        self.assertTrue(all(s.state == BALANCING for s in self.states), "fell while driving")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -6, -15])
