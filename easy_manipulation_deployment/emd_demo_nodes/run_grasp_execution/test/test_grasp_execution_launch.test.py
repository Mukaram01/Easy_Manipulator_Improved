#!/usr/bin/env python3

import os
import time
import unittest

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_testing_ros.wait_for_topics import WaitForTopics
import pytest
import rclpy


try:
    get_package_share_directory("new_scene")
except PackageNotFoundError:
    pytest.skip("new_scene package not available", allow_module_level=True)


@pytest.mark.launch_test
def generate_test_description():
    launch_path = os.path.join(
        get_package_share_directory("run_grasp_execution"),
        "launch",
        "grasp_execution.launch.py",
    )
    return (
        launch.LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                ),
                WaitForTopics([
                    ("/joint_states", "sensor_msgs/msg/JointState"),
                ]),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestGraspExecutionLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_grasp_execution_launch")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, predicate, timeout_sec=30.0):
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def _node_available(self, node_name):
        return any(
            name == node_name for name, _namespace in self.node.get_node_names_and_namespaces()
        )

    def test_required_nodes(self):
        for node_name in ["grasp_execution_node", "robot_state_publisher"]:
            self.assertTrue(self._wait_for(lambda: self._node_available(node_name)))


@launch_testing.post_shutdown_test()
class TestGraspExecutionShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
