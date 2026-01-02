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
import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers


try:
    UR5_SHARE = get_package_share_directory("ur5_2f_test")
except PackageNotFoundError:
    pytest.skip("ur5_2f_test package not available", allow_module_level=True)


@pytest.mark.launch_test
def generate_test_description():
    launch_path = os.path.join(UR5_SHARE, "launch", "demo.launch.py")
    return (
        launch.LaunchDescription(
            [
                IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path)),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestUr5DemoControllers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_ur5_2f_demo_launch")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_active_controllers(self, required_controllers, timeout_sec=60.0):
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False

        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                active = {
                    controller.name
                    for controller in future.result().controller
                    if controller.state == "active"
                }
                if required_controllers.issubset(active):
                    return True
            time.sleep(0.2)
        return False

    def test_controllers_active(self):
        required = {"joint_state_broadcaster", "ur5_arm_controller"}
        self.assertTrue(self._wait_for_active_controllers(required))


@launch_testing.post_shutdown_test()
class TestUr5DemoShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
