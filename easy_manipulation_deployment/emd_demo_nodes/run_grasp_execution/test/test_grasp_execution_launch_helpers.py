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
import sys
import types
from pathlib import Path

import pytest


launch_mod = types.ModuleType("launch")


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


launch_mod.LaunchDescription = _LaunchDescription
sys.modules.setdefault("launch", launch_mod)
launch_actions_mod = types.ModuleType("launch.actions")
launch_actions_mod.ExecuteProcess = object
launch_actions_mod.DeclareLaunchArgument = object
launch_actions_mod.TimerAction = object
launch_actions_mod.OpaqueFunction = object
launch_mod.actions = launch_actions_mod
sys.modules.setdefault("launch.actions", launch_actions_mod)
launch_substitutions_mod = types.ModuleType("launch.substitutions")
launch_substitutions_mod.LaunchConfiguration = object
launch_substitutions_mod.PythonExpression = object
sys.modules.setdefault("launch.substitutions", launch_substitutions_mod)
launch_logging_mod = types.ModuleType("launch.logging")
launch_logging_mod.get_logger = (
    lambda _name: types.SimpleNamespace(error=lambda *_args, **_kwargs: None)
)
sys.modules.setdefault("launch.logging", launch_logging_mod)
launch_ros_mod = types.ModuleType("launch_ros")
actions_mod = types.ModuleType("launch_ros.actions")
actions_mod.Node = object
launch_ros_mod.actions = actions_mod
sys.modules.setdefault("launch_ros", launch_ros_mod)
sys.modules.setdefault("launch_ros.actions", actions_mod)

aip_mod = types.ModuleType("ament_index_python")
packages_mod = types.ModuleType("ament_index_python.packages")


class PackageNotFoundError(Exception):
    pass


packages_mod.PackageNotFoundError = PackageNotFoundError
packages_mod.get_package_share_directory = lambda pkg: str(Path(pkg))
aip_mod.packages = packages_mod
sys.modules.setdefault("ament_index_python", aip_mod)
sys.modules.setdefault("ament_index_python.packages", packages_mod)

pytest.importorskip("xacro")


def import_launch_module(path: Path):
    module_name = "_launch_test_" + "_".join(path.parts[-3:])
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def grasp_launch_module():
    module_path = Path(__file__).resolve().parent.parent / "launch" / "grasp_execution.launch.py"
    return import_launch_module(module_path)


def test_find_default_scene_package_prefers_packaged_example(grasp_launch_module, monkeypatch):
    package_paths = {
        "ur5_3f_test": "/tmp/ur5_3f_test",
        "suction_test": "/tmp/suction_test",
    }

    def fake_get_package_share_directory(package_name):
        if package_name not in package_paths:
            raise grasp_launch_module.PackageNotFoundError(package_name)
        return package_paths[package_name]

    monkeypatch.setattr(
        grasp_launch_module,
        "get_package_share_directory",
        fake_get_package_share_directory,
    )

    assert grasp_launch_module.find_default_scene_package() == "ur5_3f_test"


def test_resolve_scene_package_share_dir_reports_missing_scene(grasp_launch_module, monkeypatch):
    def fake_get_package_share_directory(package_name):
        raise grasp_launch_module.PackageNotFoundError(package_name)

    monkeypatch.setattr(
        grasp_launch_module,
        "get_package_share_directory",
        fake_get_package_share_directory,
    )

    with pytest.raises(RuntimeError, match="Scene package 'missing_scene' was not found") as exc_info:
        grasp_launch_module.resolve_scene_package_share_dir("missing_scene")

    assert "scene_package:=ur5_3f_test" in str(exc_info.value)


def test_generate_launch_description_uses_safe_default_when_no_scene_is_discovered(grasp_launch_module, monkeypatch):
    captured_arguments = []

    class FakeDeclareLaunchArgument:
        def __init__(self, name, **kwargs):
            captured_arguments.append((name, kwargs))
            self.name = name
            self.kwargs = kwargs

    class FakeLaunchDescription:
        def __init__(self, entities):
            self.entities = entities

    monkeypatch.setattr(grasp_launch_module, "DeclareLaunchArgument", FakeDeclareLaunchArgument)
    monkeypatch.setattr(grasp_launch_module, "LaunchDescription", FakeLaunchDescription)
    monkeypatch.setattr(grasp_launch_module, "OpaqueFunction", lambda function: ("opaque", function))
    monkeypatch.setattr(grasp_launch_module, "DEFAULT_SCENE_PACKAGE", None)

    launch_description = grasp_launch_module.generate_launch_description()

    assert isinstance(launch_description, FakeLaunchDescription)
    scene_arguments = [
        kwargs
        for name, kwargs in captured_arguments
        if name == grasp_launch_module.SCENE_PACKAGE_ARGUMENT
    ]
    assert len(scene_arguments) == 1
    assert scene_arguments[0]["default_value"] == "ur5_3f_test"


def test_launch_setup_rejects_empty_scene_package_with_deterministic_error(grasp_launch_module, monkeypatch):
    class FakeLaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context.get(self.name)

    monkeypatch.setattr(grasp_launch_module, "LaunchConfiguration", FakeLaunchConfiguration)

    with pytest.raises(RuntimeError, match=r"Launch argument 'scene_package' was empty") as exc_info:
        grasp_launch_module.launch_setup({grasp_launch_module.SCENE_PACKAGE_ARGUMENT: ""})

    error_text = str(exc_info.value)
    assert "scene_package:=ur5_3f_test" in error_text
    assert "build/source your generated scene package first" in error_text


@pytest.mark.parametrize(
    "module_path",
    [
        Path(__file__).resolve().parent.parent / "launch" / "grasp_execution.launch.py",
        Path(__file__).resolve()
        .parents[2]
        .joinpath("run_waypoint_execution", "launch", "grasp_execution.launch.py"),
    ],
)
def test_load_file_returns_none_on_xacro_error(module_path, tmp_path, monkeypatch):
    module = import_launch_module(module_path)
    pkg_dir = tmp_path / "pkg"
    pkg_dir.mkdir()
    bad_xacro = pkg_dir / "bad.urdf.xacro"
    bad_xacro.write_text("<robot name='bad'>")

    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda pkg: str(pkg_dir),
    )

    if module_path.name == "grasp_execution.launch.py" and "run_grasp_execution" in str(module_path):
        with pytest.raises(RuntimeError, match="Failed to load robot description file"):
            module.load_file("pkg", bad_xacro.name)
    else:
        assert module.load_file("pkg", bad_xacro.name) is None


def test_align_gripper_controller_joints_normalizes_null_controller_names(grasp_launch_module):
    controllers_yaml = {"controller_names": None}
    ros2_controllers_yaml = {"controller_manager": {"ros__parameters": {}}}

    grasp_launch_module.align_gripper_controller_joints(
        controllers_yaml=controllers_yaml,
        ros2_controllers_yaml=ros2_controllers_yaml,
        gripper_controller_joints=("gripper_finger1_joint",),
        enable_gripper_controller=True,
    )

    assert controllers_yaml["controller_names"] == ["ur5_gripper_controller"]
    assert controllers_yaml["ur5_gripper_controller"]["joints"] == ["gripper_finger1_joint"]


@pytest.mark.parametrize(
    ("controllers_yaml", "ros2_controllers_yaml", "expected_error"),
    [
        (
            {"controller_names": "ur5_gripper_controller"},
            {"controller_manager": {"ros__parameters": {}}},
            r"'controller_names' must be a YAML sequence",
        ),
        (
            {"controller_names": []},
            {"controller_manager": {"ros__parameters": "not_a_mapping"}},
            r"'controller_manager\.ros__parameters' must be a YAML mapping",
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
            gripper_controller_joints=("gripper_finger1_joint",),
            enable_gripper_controller=True,
        )


def test_resolve_gripper_controller_joints_rejects_non_mapping_scene_metadata(grasp_launch_module, monkeypatch):
    monkeypatch.setattr(
        grasp_launch_module,
        "load_yaml",
        lambda package, file_path: ["not", "a", "mapping"],
    )

    with pytest.raises(RuntimeError, match=r"Invalid scene metadata schema in 'environment\.yaml'") as exc_info:
        grasp_launch_module.resolve_gripper_controller_joints("bad_scene_pkg")

    message = str(exc_info.value)
    assert "expected a YAML mapping (dict) at the root" in message
    assert "scene_package" in message


@pytest.mark.parametrize(
    "scene_package",
    [
        "ur5_2f_test",
        "ur5_3f_test",
        "suction_test",
        "ur5_airpick4_test",
        "ur3_suction_test",
        "ur10_2f_test",
    ],
)
def test_load_scene_environment_supports_known_generated_scene_packages(grasp_launch_module, monkeypatch, scene_package):
    repo_root = Path(__file__).resolve().parents[4]
    scenes_root = repo_root / "scenes"

    def fake_get_package_share_directory(package_name):
        candidate = scenes_root / package_name
        if candidate.is_dir():
            return str(candidate)
        raise grasp_launch_module.PackageNotFoundError(package_name)

    monkeypatch.setattr(grasp_launch_module, "get_package_share_directory", fake_get_package_share_directory)

    scene_metadata = grasp_launch_module.load_scene_environment(scene_package)

    assert isinstance(scene_metadata, dict)
    assert isinstance(scene_metadata.get("robot"), dict)
    assert isinstance(scene_metadata.get("end_effector"), dict)


@pytest.mark.parametrize(
    ("scene_package", "expected_brand", "expected_link"),
    [
        ("ur5_2f_test", "robotiq_2f", "gripper_base_link"),
        ("ur5_3f_test", "robotiq_3f", "palm"),
        ("suction_test", "suction_cup", "wrist_fixture"),
        ("ur5_airpick4_test", "suction_cup", "gripper_base_link"),
    ],
)
def test_build_workcell_context_maps_scene_end_effector_to_planner_brand_and_link(
    grasp_launch_module, monkeypatch, scene_package, expected_brand, expected_link
):
    repo_root = Path(__file__).resolve().parents[4]
    scenes_root = repo_root / "scenes"

    monkeypatch.setattr(
        grasp_launch_module,
        "get_package_share_directory",
        lambda package_name: str(scenes_root / package_name),
    )

    scene_metadata = grasp_launch_module.load_scene_environment(scene_package)
    workcell_context, ee_id, ee_link = grasp_launch_module.build_workcell_context_for_scene(
        scene_package, scene_metadata
    )

    ros_params = workcell_context["workcell"]["ros__parameters"]
    assert ee_id == expected_brand
    assert ee_link == expected_link
    assert ros_params["groups.manipulator.end_effectors"] == [expected_brand]
    assert ros_params[f"groups.manipulator.end_effectors.{expected_brand}.brand"] == expected_brand
    assert ros_params[f"groups.manipulator.end_effectors.{expected_brand}.link"] == expected_link


def test_build_workcell_context_falls_back_to_ur_tool0_only_when_scene_has_no_end_effector(grasp_launch_module):
    workcell_context, ee_id, ee_link = grasp_launch_module.build_workcell_context_for_scene(
        "scene_without_ee",
        {"robot": {"name": "ur5"}},
    )

    ros_params = workcell_context["workcell"]["ros__parameters"]
    assert ee_id == "ur_tool0"
    assert ee_link == "tool0"
    assert ros_params["groups.manipulator.end_effectors"] == ["ur_tool0"]



def test_require_yaml_mapping_reports_package_and_file(grasp_launch_module):
    with pytest.raises(
        RuntimeError,
        match=(
            r"Invalid YAML schema for package 'ur5_moveit_config', file 'config/kinematics\.yaml': "
            r"'robot_description_kinematics' must be a YAML mapping \(dict\), got list"
        ),
    ):
        grasp_launch_module.require_yaml_mapping(
            ["invalid"],
            "robot_description_kinematics",
            "ur5_moveit_config",
            "config/kinematics.yaml",
        )


def test_launch_setup_wraps_scene_metadata_parse_errors_with_argument_context(grasp_launch_module, monkeypatch):
    class FakeLaunchConfiguration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    monkeypatch.setattr(grasp_launch_module, "LaunchConfiguration", FakeLaunchConfiguration)
    monkeypatch.setattr(grasp_launch_module, "get_package_share_directory", lambda _package: "/tmp/fake_share")
    monkeypatch.setattr(grasp_launch_module, "resolve_scene_package_share_dir", lambda _scene_package: "/tmp/fake")
    monkeypatch.setattr(grasp_launch_module, "resolve_required_package_share_dir", lambda *_args, **_kwargs: "/tmp/fake")
    monkeypatch.setattr(
        grasp_launch_module,
        "resolve_gripper_controller_joints",
        lambda _scene_package: (_ for _ in ()).throw(RuntimeError("bad environment yaml")),
    )

    with pytest.raises(RuntimeError, match=r"Failed while parsing scene metadata for grasp execution launch") as exc_info:
        grasp_launch_module.launch_setup(
            {
                grasp_launch_module.SCENE_PACKAGE_ARGUMENT: "scene_pkg",
                grasp_launch_module.MOVEIT_CONFIG_PACKAGE_ARGUMENT: "moveit_pkg",
                grasp_launch_module.PLANNING_FRAME_ARGUMENT: "world",
            }
        )

    message = str(exc_info.value)
    assert "package='scene_pkg'" in message
    assert "file='environment.yaml'" in message
    assert "scene_package='scene_pkg'" in message
    assert "moveit_config_package='moveit_pkg'" in message
    assert "planning_frame='world'" in message
