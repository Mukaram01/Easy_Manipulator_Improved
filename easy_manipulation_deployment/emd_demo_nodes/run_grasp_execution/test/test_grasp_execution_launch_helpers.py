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
"""
Unit tests for ``grasp_execution.launch.py`` helper functions.

This module is a pytest unit-test helper and **not** a ROS launch entrypoint.
It intentionally stubs ROS-specific modules so the tests can run in a plain
Python environment.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import re
import sys
import types

import pytest


launch_mod = types.ModuleType('launch')


class _LaunchDescription:

    def __init__(self, entities):
        self.entities = entities


launch_mod.LaunchDescription = _LaunchDescription
sys.modules.setdefault('launch', launch_mod)
launch_actions_mod = types.ModuleType('launch.actions')
launch_actions_mod.ExecuteProcess = object
launch_actions_mod.DeclareLaunchArgument = object
launch_actions_mod.TimerAction = object
launch_actions_mod.OpaqueFunction = object
launch_mod.actions = launch_actions_mod
sys.modules.setdefault('launch.actions', launch_actions_mod)
launch_substitutions_mod = types.ModuleType('launch.substitutions')
launch_substitutions_mod.LaunchConfiguration = object
launch_substitutions_mod.PythonExpression = object
sys.modules.setdefault('launch.substitutions', launch_substitutions_mod)
launch_logging_mod = types.ModuleType('launch.logging')
launch_logging_mod.get_logger = (
    lambda _name: types.SimpleNamespace(
        error=lambda *_args, **_kwargs: None,
        warning=lambda *_args, **_kwargs: None,
        info=lambda *_args, **_kwargs: None,
    )
)
sys.modules.setdefault('launch.logging', launch_logging_mod)
launch_ros_mod = types.ModuleType('launch_ros')
actions_mod = types.ModuleType('launch_ros.actions')
actions_mod.Node = object
launch_ros_mod.actions = actions_mod
sys.modules.setdefault('launch_ros', launch_ros_mod)
sys.modules.setdefault('launch_ros.actions', actions_mod)

aip_mod = types.ModuleType('ament_index_python')
packages_mod = types.ModuleType('ament_index_python.packages')


class PackageNotFoundError(Exception):
    pass


packages_mod.PackageNotFoundError = PackageNotFoundError
packages_mod.get_package_share_directory = lambda pkg: str(Path(pkg))
aip_mod.packages = packages_mod
sys.modules.setdefault('ament_index_python', aip_mod)
sys.modules.setdefault('ament_index_python.packages', packages_mod)

pytest.importorskip('xacro')


def import_launch_module(path: Path):
    module_name = '_launch_test_' + '_'.join(path.parts[-3:])
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f'Cannot load module from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def grasp_launch_module():
    module_path = Path(__file__).resolve().parent.parent / 'launch' / 'grasp_execution.launch.py'
    return import_launch_module(module_path)


def test_find_default_scene_package_prefers_packaged_example(grasp_launch_module, monkeypatch):
    package_paths = {
        'ur5_3f_test': '/tmp/ur5_3f_test',
        'suction_test': '/tmp/suction_test',
    }

    def fake_get_package_share_directory(package_name):
        if package_name not in package_paths:
            raise grasp_launch_module.PackageNotFoundError(package_name)
        return package_paths[package_name]

    monkeypatch.setattr(
        grasp_launch_module,
        'get_package_share_directory',
        fake_get_package_share_directory,
    )

    assert grasp_launch_module.find_default_scene_package() == 'ur5_3f_test'


def test_resolve_scene_package_share_dir_reports_missing_scene(grasp_launch_module, monkeypatch):
    def fake_get_package_share_directory(package_name):
        raise grasp_launch_module.PackageNotFoundError(package_name)

    monkeypatch.setattr(
        grasp_launch_module,
        'get_package_share_directory',
        fake_get_package_share_directory,
    )

    with pytest.raises(
        RuntimeError,
        match='Scene package \x27missing_scene\x27 was not found',
    ) as exc_info:
        grasp_launch_module.resolve_scene_package_share_dir('missing_scene')

    assert 'scene_package:=ur5_3f_test' in str(exc_info.value)


def test_generate_launch_description_uses_safe_default_when_no_scene_is_discovered(
    grasp_launch_module,
    monkeypatch,
):
    captured_arguments = []

    class FakeDeclareLaunchArgument:

        def __init__(self, name, **kwargs):
            captured_arguments.append((name, kwargs))
            self.name = name
            self.kwargs = kwargs

    class FakeLaunchDescription:

        def __init__(self, entities):
            self.entities = entities

    monkeypatch.setattr(
        grasp_launch_module,
        'DeclareLaunchArgument',
        FakeDeclareLaunchArgument,
    )
    monkeypatch.setattr(grasp_launch_module, 'LaunchDescription', FakeLaunchDescription)
    monkeypatch.setattr(
        grasp_launch_module,
        'OpaqueFunction',
        lambda function: ('opaque', function),
    )
    monkeypatch.setattr(grasp_launch_module, 'DEFAULT_SCENE_PACKAGE', None)

    launch_description = grasp_launch_module.generate_launch_description()

    assert isinstance(launch_description, FakeLaunchDescription)
    scene_arguments = [
        kwargs
        for name, kwargs in captured_arguments
        if name == grasp_launch_module.SCENE_PACKAGE_ARGUMENT
    ]
    assert len(scene_arguments) == 1
    assert scene_arguments[0]['default_value'] == 'ur5_3f_test'


def test_launch_setup_rejects_empty_scene_package_with_deterministic_error(
    grasp_launch_module,
    monkeypatch,
):

    class FakeLaunchConfiguration:

        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context.get(self.name)

    monkeypatch.setattr(
        grasp_launch_module,
        'LaunchConfiguration',
        FakeLaunchConfiguration,
    )

    with pytest.raises(
        RuntimeError,
        match=r'Launch argument \'scene_package\' was empty',
    ) as exc_info:
        grasp_launch_module.launch_setup({grasp_launch_module.SCENE_PACKAGE_ARGUMENT: ''})

    error_text = str(exc_info.value)
    assert 'scene_package:=ur5_3f_test' in error_text
    assert 'build/source your generated scene package first' in error_text


@pytest.mark.parametrize(
    'module_path',
    [
        Path(__file__).resolve().parent.parent / 'launch' / 'grasp_execution.launch.py',
        Path(__file__).resolve()
        .parents[2]
        .joinpath('run_waypoint_execution', 'launch', 'grasp_execution.launch.py'),
    ],
)
def test_load_file_returns_none_on_xacro_error(module_path, tmp_path, monkeypatch):
    module = import_launch_module(module_path)
    pkg_dir = tmp_path / 'pkg'
    pkg_dir.mkdir()
    bad_xacro = pkg_dir / 'bad.urdf.xacro'
    bad_xacro.write_text('<robot name=\x27bad\x27>')

    monkeypatch.setattr(
        module,
        'get_package_share_directory',
        lambda pkg: str(pkg_dir),
    )

    is_grasp_execution_launch = (
        module_path.name == 'grasp_execution.launch.py'
        and 'run_grasp_execution' in str(module_path)
    )
    if is_grasp_execution_launch:
        with pytest.raises(RuntimeError, match='Failed to load robot description file'):
            module.load_file('pkg', bad_xacro.name)
    else:
        assert module.load_file('pkg', bad_xacro.name) is None


def test_align_gripper_controller_joints_normalizes_null_controller_names(grasp_launch_module):
    controllers_yaml = {'controller_names': None}
    ros2_controllers_yaml = {'controller_manager': {'ros__parameters': {}}}

    grasp_launch_module.align_gripper_controller_joints(
        controllers_yaml=controllers_yaml,
        ros2_controllers_yaml=ros2_controllers_yaml,
        gripper_controller_joints=('gripper_finger1_joint',),
        enable_gripper_controller=True,
    )

    assert controllers_yaml['controller_names'] == ['ur5_gripper_controller']
    assert controllers_yaml['ur5_gripper_controller']['joints'] == ['gripper_finger1_joint']


def test_validate_gripper_controller_consistency_enables_when_joints_and_interfaces_match(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <joint name="gripper_finger1_joint" type="revolute"/>
      <ros2_control name="GripperSystem" type="system">
        <joint name="gripper_finger1_joint">
          <command_interface name="position"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
      </ros2_control>
    </robot>
    """
    ros2_controllers_yaml = {
        'ur5_gripper_controller': {
            'ros__parameters': {
                'joints': ['gripper_finger1_joint'],
            }
        }
    }

    enabled, details = grasp_launch_module.validate_gripper_controller_consistency(
        robot_description_xml=robot_description_xml,
        ros2_controllers_yaml=ros2_controllers_yaml,
        gripper_controller_joints=('gripper_finger1_joint',),
    )

    assert enabled is True
    assert details['robot_description']['missing_robot_description_joints'] == []
    assert details['robot_description']['missing_command_interfaces'] == {}
    assert details['robot_description']['missing_state_interfaces'] == {}
    assert details['missing_ros2_controller_joints'] == []


def test_home_return_safe_intermediate_default_is_disabled():
    config_path = Path(__file__).resolve().parent.parent / 'config' / 'grasp_execution.yaml'
    config_text = config_path.read_text(encoding='utf-8')
    assert re.search(
        r'home_return:\s*\n(?:\s+.+\n)*?\s+use_safe_intermediate:\s*false\b',
        config_text,
    )


def test_validate_gripper_controller_consistency_disables_when_joints_missing(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <joint name="wrist_3_joint" type="revolute"/>
    </robot>
    """
    ros2_controllers_yaml = {
        'ur5_gripper_controller': {'ros__parameters': {'joints': ['gripper_finger1_joint']}}
    }

    enabled, details = grasp_launch_module.validate_gripper_controller_consistency(
        robot_description_xml=robot_description_xml,
        ros2_controllers_yaml=ros2_controllers_yaml,
        gripper_controller_joints=('gripper_finger1_joint',),
    )

    assert enabled is False
    assert details['robot_description']['missing_robot_description_joints'] == [
        'gripper_finger1_joint'
    ]


def test_validate_gripper_controller_consistency_disables_when_interfaces_missing(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <joint name="gripper_finger1_joint" type="revolute"/>
      <ros2_control name="GripperSystem" type="system">
        <joint name="gripper_finger1_joint">
          <command_interface name="effort"/>
          <state_interface name="position"/>
        </joint>
      </ros2_control>
    </robot>
    """
    ros2_controllers_yaml = {
        'ur5_gripper_controller': {'ros__parameters': {'joints': ['gripper_finger1_joint']}}
    }

    enabled, details = grasp_launch_module.validate_gripper_controller_consistency(
        robot_description_xml=robot_description_xml,
        ros2_controllers_yaml=ros2_controllers_yaml,
        gripper_controller_joints=('gripper_finger1_joint',),
    )

    assert enabled is False
    assert details['robot_description']['missing_command_interfaces'] == {
        'gripper_finger1_joint': ['position']
    }
    assert details['robot_description']['missing_state_interfaces'] == {
        'gripper_finger1_joint': ['velocity']
    }


@pytest.mark.parametrize(
    ('controllers_yaml', 'ros2_controllers_yaml', 'expected_error'),
    [
        (
            {'controller_names': 'ur5_gripper_controller'},
            {'controller_manager': {'ros__parameters': {}}},
            r'\'controller_names\' must be a YAML sequence',
        ),
        (
            {'controller_names': []},
            {'controller_manager': {'ros__parameters': 'not_a_mapping'}},
            r'\'controller_manager\.ros__parameters\' must be a YAML mapping',
        ),
    ],
)
def test_align_gripper_controller_joints_rejects_invalid_schema_types(
    grasp_launch_module,
    controllers_yaml,
    ros2_controllers_yaml,
    expected_error,
):
    with pytest.raises(RuntimeError, match=expected_error):
        grasp_launch_module.align_gripper_controller_joints(
            controllers_yaml=controllers_yaml,
            ros2_controllers_yaml=ros2_controllers_yaml,
            gripper_controller_joints=('gripper_finger1_joint',),
            enable_gripper_controller=True,
        )


def test_resolve_gripper_controller_joints_rejects_non_mapping_scene_metadata(
    grasp_launch_module,
    monkeypatch,
):
    monkeypatch.setattr(
        grasp_launch_module,
        'load_yaml',
        lambda package, file_path: ['not', 'a', 'mapping'],
    )

    with pytest.raises(
        RuntimeError,
        match=r'Invalid scene metadata schema in \'environment\.yaml\'',
    ) as exc_info:
        grasp_launch_module.resolve_gripper_controller_joints('bad_scene_pkg')

    message = str(exc_info.value)
    assert 'expected a YAML mapping (dict) at the root' in message
    assert 'scene_package' in message


@pytest.mark.parametrize(
    'scene_package',
    [
        'ur5_2f_test',
        'ur5_3f_test',
        'suction_test',
        'ur5_airpick4_test',
        'ur3_suction_test',
        'ur10_2f_test',
    ],
)
def test_load_scene_environment_supports_known_generated_scene_packages(
    grasp_launch_module,
    monkeypatch,
    scene_package,
):
    repo_root = Path(__file__).resolve().parents[4]
    scenes_root = repo_root / 'scenes'

    def fake_get_package_share_directory(package_name):
        candidate = scenes_root / package_name
        if candidate.is_dir():
            return str(candidate)
        raise grasp_launch_module.PackageNotFoundError(package_name)

    monkeypatch.setattr(
        grasp_launch_module,
        'get_package_share_directory',
        fake_get_package_share_directory,
    )

    scene_metadata = grasp_launch_module.load_scene_environment(scene_package)

    assert isinstance(scene_metadata, dict)
    assert isinstance(scene_metadata.get('robot'), dict)
    assert isinstance(scene_metadata.get('end_effector'), dict)


@pytest.mark.parametrize(
    ('scene_package', 'expected_brand', 'expected_moveit_link', 'expected_grasp_frame'),
    [
        ('ur5_2f_test', 'robotiq_2f', 'tool0', 'ee_palm'),
        ('ur5_3f_test', 'ur_tool0', 'tool0', 'tool0'),
        ('suction_test', 'suction_cup', 'tool0', 'wrist_fixture'),
        ('ur5_airpick4_test', 'suction_cup', 'tool0', 'gripper_base_link'),
    ],
)
def test_build_workcell_context_maps_scene_end_effector_to_planner_brand_and_link(
    grasp_launch_module,
    monkeypatch,
    scene_package,
    expected_brand,
    expected_moveit_link,
    expected_grasp_frame,
):
    repo_root = Path(__file__).resolve().parents[4]
    scenes_root = repo_root / 'scenes'

    monkeypatch.setattr(
        grasp_launch_module,
        'get_package_share_directory',
        lambda package_name: str(scenes_root / package_name),
    )

    scene_metadata = grasp_launch_module.load_scene_environment(scene_package)
    (
        workcell_context,
        ee_id,
        ee_link,
        ee_grasp_frame,
    ) = grasp_launch_module.build_workcell_context_for_scene(
        scene_package,
        scene_metadata,
    )

    ros_params = workcell_context['workcell']['ros__parameters']
    assert ee_id == expected_brand
    assert ee_link == expected_moveit_link
    assert ee_grasp_frame == expected_grasp_frame
    assert ros_params['groups.manipulator.end_effectors'] == [expected_brand]
    assert ros_params[f'groups.manipulator.end_effectors.{expected_brand}.brand'] == expected_brand
    assert (
        ros_params[f'groups.manipulator.end_effectors.{expected_brand}.link']
        == expected_moveit_link
    )
    assert (
        ros_params[f'groups.manipulator.end_effectors.{expected_brand}.grasp_frame']
        == expected_grasp_frame
    )


def test_derive_workcell_grasp_frame_prefers_ee_palm_for_robotiq_85_two_finger_metadata(
    grasp_launch_module,
):
    end_effector = {
        'name': 'custom_gripper',
        'brand': 'robotiq_85',
        'ee_type': 'parallel',
        'attributes': {'fingers': 2},
        'base_link': 'gripper_base_link',
    }

    assert grasp_launch_module.derive_workcell_grasp_frame(end_effector) == 'ee_palm'


@pytest.mark.parametrize(
    ('end_effector', 'expected_grasp_frame'),
    [
        (
            {
                'name': 'single_suction_gripper',
                'brand': 'suction',
                'links': ['wrist_3_link', 'suction_cup_link', 'airpick_tcp'],
                'tcp_link': 'airpick_tcp',
            },
            'suction_cup_link',
        ),
        (
            {
                'name': 'onrobot_airpick4',
                'brand': 'airpick',
                'links': ['airpick_base', 'airpick_tcp'],
                'tcp_link': 'airpick_tcp',
            },
            'airpick_tcp',
        ),
        (
            {
                'name': 'robotiq_3f',
                'attributes': {'fingers': 3},
                'links': ['finger_1_link_0', 'palm', 'finger_middle_link_0'],
            },
            'palm',
        ),
        (
            {
                'name': 'robotiq_3f',
                'attributes': {'fingers': 3},
                'grasp_frame': 'custom_3f_tcp',
                'links': ['palm', 'finger_1_link_0'],
            },
            'custom_3f_tcp',
        ),
    ],
)
def test_derive_workcell_grasp_frame_metadata_only_precedence(
    grasp_launch_module,
    end_effector,
    expected_grasp_frame,
):
    assert grasp_launch_module.derive_workcell_grasp_frame(end_effector) == expected_grasp_frame


def test_derive_planner_end_effector_id_maps_airpick_to_suction_cup(grasp_launch_module):
    end_effector = {'name': 'airpick', 'brand': 'onrobot'}

    assert grasp_launch_module.derive_planner_end_effector_id(end_effector) == 'suction_cup'


def test_build_workcell_context_falls_back_to_ur_tool0_when_scene_has_no_ee(
    grasp_launch_module,
):
    (
        workcell_context,
        ee_id,
        ee_link,
        ee_grasp_frame,
    ) = grasp_launch_module.build_workcell_context_for_scene(
        'scene_without_ee',
        {'robot': {'name': 'ur5'}},
    )

    ros_params = workcell_context['workcell']['ros__parameters']
    assert ee_id == 'ur_tool0'
    assert ee_link == 'tool0'
    assert ee_grasp_frame == 'tool0'
    assert ros_params['groups.manipulator.end_effectors'] == ['ur_tool0']


def test_build_workcell_context_logs_warning_for_unknown_ee_metadata_fallback(
    grasp_launch_module,
):
    logger = types.SimpleNamespace(warning_messages=[])
    logger.warning = lambda msg: logger.warning_messages.append(msg)

    unknown_scene_metadata = {
        'robot': {'name': 'ur5'},
        'end_effector': {
            'name': ' mystery_hand ',
            'brand': ' ACME ',
            'ee_type': ' HybridParallel ',
            'robot_link': 'tool0',
        },
    }

    (
        _workcell_context,
        ee_id,
        ee_link,
        ee_grasp_frame,
    ) = grasp_launch_module.build_workcell_context_for_scene(
        'mystery_scene',
        unknown_scene_metadata,
        logger=logger,
    )

    assert (ee_id, ee_link, ee_grasp_frame) == ('ur_tool0', 'tool0', 'tool0')
    assert len(logger.warning_messages) == 1
    assert 'mystery_scene' in logger.warning_messages[0]
    assert 'name=\x27mystery_hand\x27' in logger.warning_messages[0]
    assert 'brand=\x27acme\x27' in logger.warning_messages[0]
    assert 'ee_type=\x27hybridparallel\x27' in logger.warning_messages[0]
    assert 'tool0' in logger.warning_messages[0]


@pytest.fixture()
def three_finger_metadata_with_arm_only_urdf():
    scene_metadata = {
        'robot': {'name': 'ur5'},
        'end_effector': {
            'name': 'robotiq_3f',
            'brand': 'robotiq_3f_gripper',
            'robot_link': 'tool0',
            'base_link': 'palm',
            'attributes': {'fingers': 3},
        },
    }
    arm_only_links = {'world', 'base_link', 'wrist_3_link', 'tool0'}
    return scene_metadata, arm_only_links


def test_validate_and_normalize_workcell_end_effector_frames_falls_back_on_3f_urdf_mismatch(
    grasp_launch_module,
    three_finger_metadata_with_arm_only_urdf,
):
    scene_metadata, arm_only_links = three_finger_metadata_with_arm_only_urdf
    logger = types.SimpleNamespace(warning_messages=[])
    logger.warning = lambda msg: logger.warning_messages.append(msg)

    (
        ee_id,
        ee_link,
        ee_grasp_frame,
    ) = grasp_launch_module.validate_and_normalize_workcell_end_effector_frames(
        scene_metadata=scene_metadata,
        ee_id='robotiq_3f',
        ee_link='tool0',
        ee_grasp_frame='palm',
        link_names=arm_only_links,
        logger=logger,
    )

    assert (ee_id, ee_link, ee_grasp_frame) == ('ur_tool0', 'tool0', 'tool0')
    assert any(
        'declares Robotiq 3F' in message for message in logger.warning_messages
    )


def _sorted_pair_set(triples):
    return {tuple(sorted((link1, link2))) for link1, link2, _reason in triples}


def test_compute_conservative_srdf_disable_collision_injections_keeps_ur_base_and_forearm_wrist2(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <link name="base_link"/>
      <link name="base_link_inertia"/>
      <link name="shoulder_link"/>
      <link name="forearm_link"/>
      <link name="wrist_2_link"/>
    </robot>
    """
    srdf_xml = '<robot name=\x27demo\x27></robot>'

    injections = grasp_launch_module._compute_conservative_srdf_disable_collision_injections(
        robot_description_xml,
        srdf_xml,
    )
    pair_set = _sorted_pair_set(injections)

    assert tuple(sorted(('base_link', 'base_link_inertia'))) in pair_set
    assert tuple(sorted(('base_link_inertia', 'shoulder_link'))) in pair_set
    assert tuple(sorted(('forearm_link', 'wrist_2_link'))) in pair_set


def test_conservative_srdf_injections_adds_fixed_camera_wrist_pair(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <link name="wrist_3_link"/>
      <link name="camera_bottom_screw_frame"/>
      <joint name="wrist_to_camera_mount" type="fixed">
        <parent link="wrist_3_link"/>
        <child link="camera_bottom_screw_frame"/>
      </joint>
    </robot>
    """
    srdf_xml = '<robot name=\x27demo\x27></robot>'

    injections = grasp_launch_module._compute_conservative_srdf_disable_collision_injections(
        robot_description_xml,
        srdf_xml,
    )

    wrist_camera_pair = tuple(
        sorted(('wrist_3_link', 'camera_bottom_screw_frame'))
    )
    assert wrist_camera_pair in _sorted_pair_set(injections)


def test_conservative_srdf_injections_rejects_non_fixed_camera_wrist_pair(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <link name="wrist_3_link"/>
      <link name="camera_bottom_screw_frame"/>
      <joint name="wrist_to_camera_mount" type="revolute">
        <parent link="wrist_3_link"/>
        <child link="camera_bottom_screw_frame"/>
      </joint>
    </robot>
    """
    srdf_xml = '<robot name=\x27demo\x27></robot>'

    injections = grasp_launch_module._compute_conservative_srdf_disable_collision_injections(
        robot_description_xml,
        srdf_xml,
    )

    wrist_camera_pair = tuple(
        sorted(('wrist_3_link', 'camera_bottom_screw_frame'))
    )
    assert wrist_camera_pair not in _sorted_pair_set(injections)


def test_conservative_srdf_injections_never_adds_table_or_workbench_pairs(
    grasp_launch_module,
):
    robot_description_xml = """
    <robot name="demo">
      <link name="wrist_3_link"/>
      <link name="table_top"/>
      <joint name="table_joint" type="fixed">
        <parent link="wrist_3_link"/>
        <child link="table_top"/>
      </joint>
    </robot>
    """
    srdf_xml = '<robot name=\x27demo\x27></robot>'

    injections = grasp_launch_module._compute_conservative_srdf_disable_collision_injections(
        robot_description_xml,
        srdf_xml,
    )

    for link1, link2, _reason in injections:
        pair_text = f'{link1} {link2}'.lower()
        assert 'table' not in pair_text
        assert 'workbench' not in pair_text


def test_require_yaml_mapping_reports_package_and_file(grasp_launch_module):
    with pytest.raises(
        RuntimeError,
        match=(
            r'Invalid YAML schema for package \'ur5_moveit_config\', '
            r'file \'config/kinematics\.yaml\': '
            r'\'robot_description_kinematics\' must be a YAML mapping \(dict\), got list'
        ),
    ):
        grasp_launch_module.require_yaml_mapping(
            ['invalid'],
            'robot_description_kinematics',
            'ur5_moveit_config',
            'config/kinematics.yaml',
        )


def test_launch_setup_wraps_scene_metadata_parse_errors_with_argument_context(
    grasp_launch_module,
    monkeypatch,
):

    class FakeLaunchConfiguration:

        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    monkeypatch.setattr(
        grasp_launch_module,
        'LaunchConfiguration',
        FakeLaunchConfiguration,
    )
    monkeypatch.setattr(
        grasp_launch_module,
        'get_package_share_directory',
        lambda _package: '/tmp/fake_share',
    )
    monkeypatch.setattr(
        grasp_launch_module,
        'resolve_scene_package_share_dir',
        lambda _scene_package: '/tmp/fake',
    )
    monkeypatch.setattr(
        grasp_launch_module,
        'resolve_required_package_share_dir',
        lambda *_args, **_kwargs: '/tmp/fake',
    )
    monkeypatch.setattr(
        grasp_launch_module,
        'resolve_gripper_controller_joints',
        lambda _scene_package: (_ for _ in ()).throw(RuntimeError('bad environment yaml')),
    )

    with pytest.raises(
        RuntimeError,
        match=r'Failed while parsing scene metadata for grasp execution launch',
    ) as exc_info:
        grasp_launch_module.launch_setup(
            {
                grasp_launch_module.SCENE_PACKAGE_ARGUMENT: 'scene_pkg',
                grasp_launch_module.MOVEIT_CONFIG_PACKAGE_ARGUMENT: 'moveit_pkg',
                grasp_launch_module.PLANNING_FRAME_ARGUMENT: 'world',
            }
        )

    message = str(exc_info.value)
    assert 'package=\x27scene_pkg\x27' in message
    assert 'file=\x27environment.yaml\x27' in message
    assert 'scene_package=\x27scene_pkg\x27' in message
    assert 'moveit_config_package=\x27moveit_pkg\x27' in message
    assert 'planning_frame=\x27world\x27' in message
