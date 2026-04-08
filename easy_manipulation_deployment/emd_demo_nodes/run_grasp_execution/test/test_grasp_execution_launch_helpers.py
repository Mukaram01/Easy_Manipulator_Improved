"""Unit tests for ``grasp_execution.launch.py`` helper functions.

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
launch_logging_mod.get_logger = lambda _name: types.SimpleNamespace(error=lambda *_args, **_kwargs: None)
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
