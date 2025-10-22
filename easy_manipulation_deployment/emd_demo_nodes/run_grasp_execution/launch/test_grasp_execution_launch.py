"""Tests for the grasp_execution launch utilities.

These tests focus on the helper functions defined in
``grasp_execution.launch.py``.  They intentionally stub out ROS specific
modules so they can run in a plain Python environment.
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest


# The launch file imports a number of ROS specific modules.  Stub them out so
# the module can be imported without ROS being available in the test
# environment.
launch_mod = types.ModuleType("launch")
launch_mod.LaunchDescription = object
sys.modules.setdefault("launch", launch_mod)
launch_actions_mod = types.ModuleType("launch.actions")
launch_actions_mod.ExecuteProcess = object
launch_actions_mod.DeclareLaunchArgument = object
launch_mod.actions = launch_actions_mod
sys.modules.setdefault("launch.actions", launch_actions_mod)
launch_substitutions_mod = types.ModuleType("launch.substitutions")
launch_substitutions_mod.LaunchConfiguration = object
launch_substitutions_mod.PythonExpression = object
sys.modules.setdefault("launch.substitutions", launch_substitutions_mod)
launch_ros_mod = types.ModuleType("launch_ros")
actions_mod = types.ModuleType("launch_ros.actions")
actions_mod.Node = object
launch_ros_mod.actions = actions_mod
sys.modules.setdefault("launch_ros", launch_ros_mod)
sys.modules.setdefault("launch_ros.actions", actions_mod)

# ``grasp_execution.launch.py`` relies on ``ament_index_python`` to resolve
# package directories.  Provide a thin shim so tests can control the returned
# paths via ``monkeypatch``.
aip_mod = types.ModuleType("ament_index_python")
packages_mod = types.ModuleType("ament_index_python.packages")
packages_mod.get_package_share_directory = lambda pkg: str(Path(pkg))
aip_mod.packages = packages_mod
sys.modules.setdefault("ament_index_python", aip_mod)
sys.modules.setdefault("ament_index_python.packages", packages_mod)

pytest.importorskip("xacro")


def import_launch_module(path: Path):
    """Import a launch file as a Python module."""

    module_name = "_launch_test_" + "_".join(path.parts[-3:])
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "module_path",
    [
        Path(__file__).resolve().parent / "grasp_execution.launch.py",
        Path(__file__).resolve()
        .parents[2]
        .joinpath("run_waypoint_execution", "launch", "grasp_execution.launch.py"),
    ],
)
def test_load_file_returns_none_on_xacro_error(module_path, tmp_path, monkeypatch):
    """Invalid xacro input should not cause an exception to escape."""
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

    # ``load_file`` previously only caught ``EnvironmentError`` which meant
    # ``xacro.XacroException`` bubbled up for malformed files.  Ensure it now
    # gracefully handles the error and returns ``None`` instead.
    assert module.load_file("pkg", bad_xacro.name) is None
