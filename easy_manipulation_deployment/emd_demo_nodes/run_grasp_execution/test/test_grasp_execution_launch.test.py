#!/usr/bin/env python3
# Copyright 2020 ROS Industrial Consortium Asia Pacific
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

import importlib.util
import os
import time
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState


SCENE_PACKAGE = 'ur5_2f_test'


REQUIRED_GRIPPER_JOINTS = {'gripper_finger1_joint'}

try:
    get_package_share_directory(SCENE_PACKAGE)
except PackageNotFoundError:
    pytest.skip(f"{SCENE_PACKAGE} package not available", allow_module_level=True)


def import_launch_module():
    launch_path = Path(get_package_share_directory('run_grasp_execution')) / 'launch' / 'grasp_execution.launch.py'
    spec = importlib.util.spec_from_file_location('run_grasp_execution_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_missing_scene_package_error_message():
    module = import_launch_module()
    missing_scene = 'missing_scene_package'
    with pytest.raises(RuntimeError, match=rf"Scene package '{missing_scene}' was not found") as exc_info:
        module.resolve_scene_package_share_dir(missing_scene)
    assert "build/source your generated scene package first" in str(exc_info.value)


def test_scene_srdf_injections_are_scoped_and_exclude_table_workbench_rules():
    module = import_launch_module()
    scene_package = SCENE_PACKAGE
    robot_description_xml = module.load_file(scene_package, 'urdf/scene.urdf.xacro')
    srdf_xml = module.load_file(scene_package, 'urdf/arm_hand.srdf.xacro')

    pairs_to_inject = module._compute_conservative_srdf_disable_collision_injections(
        robot_description_xml=robot_description_xml,
        srdf_xml=srdf_xml,
    )
    allowed_pairs = {
        tuple(sorted(("base_link", "base_link_inertia"))),
        tuple(sorted(("base_link_inertia", "shoulder_link"))),
        tuple(sorted(("forearm_link", "wrist_2_link"))),
    }

    injected_pair_set = {tuple(sorted((link1, link2))) for link1, link2, _reason in pairs_to_inject}
    assert injected_pair_set.issubset(allowed_pairs)
    for link1, link2, _reason in pairs_to_inject:
        text = f"{link1} {link2}".lower()
        assert "table" not in text
        assert "workbench" not in text


def test_scene_srdf_no_blanket_robot_vs_table_disable_rules():
    module = import_launch_module()
    srdf_xml = module.load_file(SCENE_PACKAGE, 'urdf/arm_hand.srdf.xacro')
    srdf_root = ET.fromstring(srdf_xml)
    for element in srdf_root.findall('disable_collisions'):
        link1 = (element.attrib.get('link1', '') or '').lower()
        link2 = (element.attrib.get('link2', '') or '').lower()
        if "table" not in link1 and "table" not in link2 and "workbench" not in link1 and "workbench" not in link2:
            continue
        assert "wrist" not in link1 and "wrist" not in link2
        assert "forearm" not in link1 and "forearm" not in link2
        assert "upper_arm" not in link1 and "upper_arm" not in link2


@pytest.mark.launch_test
def generate_test_description():
    launch_path = os.path.join(
        get_package_share_directory('run_grasp_execution'),
        'launch',
        'grasp_execution.launch.py',
    )
    return (
        launch.LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_path),
                    launch_arguments={"scene_package": SCENE_PACKAGE}.items(),
                ),
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
        self.node = rclpy.create_node('test_grasp_execution_launch')

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
        for node_name in ['grasp_execution_node', 'robot_state_publisher']:
            self.assertTrue(self._wait_for(lambda: self._node_available(node_name)))

    def test_joint_states_topic_published(self):
        self.assertTrue(
            self._wait_for(
                lambda: any(
                    topic_name == '/joint_states'
                    for topic_name, _types in self.node.get_topic_names_and_types()
                )
            )
        )

    def test_joint_state_contains_expected_gripper_joints(self):
        if not REQUIRED_GRIPPER_JOINTS:
            self.skipTest(
                'Scene does not expose gripper controller joints; '
                'skipping gripper joint assertion'
            )

        observed_joint_names = set()

        def callback(msg):
            observed_joint_names.update(msg.name)

        subscription = self.node.create_subscription(JointState, '/joint_states', callback, 10)
        self.assertTrue(self._wait_for(lambda: REQUIRED_GRIPPER_JOINTS.issubset(observed_joint_names)))
        self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestGraspExecutionShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
