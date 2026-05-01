#!/usr/bin/env python3
# Copyright 2026 Mukaram
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import time
import unittest

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import launch
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy


try:
    get_package_share_directory('run_grasp_planner')
except PackageNotFoundError:
    pytest.skip('run_grasp_planner package not available', allow_module_level=True)


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory('run_grasp_planner')
    params_file = os.path.join(package_share, 'config', 'params_2f.yaml')
    demo_node = Node(
        package='run_grasp_planner',
        executable='demo_node',
        name='grasp_planning_node',
        output='screen',
        parameters=[
            params_file,
            {'easy_perception_deployment.epd_enabled': True},
            {'easy_perception_deployment.epd_subscription_reliability': 'best_effort'},
            {'camera_parameters.point_cloud_subscription_reliability': 'best_effort'},
        ],
    )

    return (
        launch.LaunchDescription([
            demo_node,
            TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
            TimerAction(period=6.0, actions=[EmitEvent(event=Shutdown())]),
        ]),
        {'demo_node': demo_node},
    )


class TestDemoNodeParameterOverrideLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_demo_node_parameter_override_launch')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, predicate, timeout_sec=10.0):
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def test_demo_node_starts_with_epd_override(self):
        self.assertTrue(
            self._wait_for(
                lambda: any(
                    name == 'grasp_planning_node'
                    for name, _namespace in self.node.get_node_names_and_namespaces()
                )
            ),
            'Expected demo_node to stay up long enough to appear in the ROS graph.',
        )


@launch_testing.post_shutdown_test()
class TestDemoNodeParameterOverrideShutdown(unittest.TestCase):
    def test_demo_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
