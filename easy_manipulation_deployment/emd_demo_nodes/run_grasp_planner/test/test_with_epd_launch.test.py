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


for pkg in ('run_grasp_planner', 'easy_perception_deployment'):
    try:
        get_package_share_directory(pkg)
    except PackageNotFoundError:
        pytest.skip(f'{pkg} package not available', allow_module_level=True)


@pytest.mark.launch_test
def generate_test_description():
    launch_path = os.path.join(
        get_package_share_directory('run_grasp_planner'),
        'launch',
        'with_epd.launch.py',
    )
    return (
        launch.LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestWithEpdLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_with_epd_launch')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, predicate, timeout_sec=30.0):
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def test_epd_integration_smoke(self):
        def epd_node_running():
            return any(
                'easy_perception' in name
                for name, _namespace in self.node.get_node_names_and_namespaces()
            )

        def epd_topic_appeared():
            topic_names = [name for name, _types in self.node.get_topic_names_and_types()]
            return '/easy_perception_deployment/epd_localize_output' in topic_names

        self.assertTrue(
            self._wait_for(lambda: epd_node_running() or epd_topic_appeared()),
            'Expected easy_perception_deployment node or output topic to become available.',
        )


@launch_testing.post_shutdown_test()
class TestWithEpdShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
