from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "ur5_2f_test"
JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
EXPECTED_HOME = {
    "shoulder_pan_joint": 1.57,
    "shoulder_lift_joint": -2.35,
    "elbow_joint": 1.83,
    "wrist_1_joint": -1.03,
    "wrist_2_joint": -1.57,
    "wrist_3_joint": 0.0,
}


def _load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _assert_joint_map_close(actual, expected, *, abs_tol=1e-9):
    assert set(actual) == set(expected)
    for name in JOINT_ORDER:
        assert math.isclose(float(actual[name]), float(expected[name]), rel_tol=0.0, abs_tol=abs_tol), name


def test_canonical_environment_has_deliberate_industrial_home_pose():
    environment = _load_yaml(SCENE / "environment.yaml")
    robot = environment["robot"]
    home = robot["home_joint_state"]

    assert home["source"] == "canonical_scene"
    _assert_joint_map_close(home["joints"], EXPECTED_HOME)

    # The home posture must not regress to either the all-zero fake-hardware state
    # or the previous elbow-straight posture that visually collapsed onto the table.
    assert not all(abs(float(home["joints"][name])) < 1e-9 for name in JOINT_ORDER)
    assert abs(float(home["joints"]["elbow_joint"])) > 1.0
    assert abs(float(home["joints"]["wrist_2_joint"])) > 1.0


def test_safe_return_and_cell_definition_match_canonical_home():
    environment = _load_yaml(SCENE / "environment.yaml")
    cell_definition = _load_yaml(SCENE / "cell_definition.yaml")
    robot = environment["robot"]
    ordered_home = [float(robot["home_joint_state"]["joints"][name]) for name in robot["joint_names"]]

    assert len(ordered_home) == 6
    for actual, expected in zip(robot["safe_joint_state"], ordered_home):
        assert math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-9)
    for actual, expected in zip(cell_definition["robot"]["safe_joint_state"], ordered_home):
        assert math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-9)


def test_scene_xacro_drives_fake_hardware_initial_state_from_environment_home():
    xacro = (SCENE / "urdf" / "scene.urdf.xacro").read_text(encoding="utf-8")

    # The launch path expands this file from the installed package. Resolve the
    # scene-local environment through the ROS package index: `$(dirname)` cannot
    # be substituted by this xacro invocation because it has no directory
    # substitution context.
    assert 'name="environment_file" default="$(find ur5_2f_test)/environment.yaml"' in xacro
    assert 'default="$(dirname)/../environment.yaml"' not in xacro
    assert 'xacro.load_yaml(environment_file)' in xacro
    assert "['robot']['home_joint_state']['joints']" in xacro
    assert 'initial_positions="${workcell_home_joint_state}"' in xacro


def test_product_view_preview_home_matches_canonical_scene_home():
    environment = _load_yaml(SCENE / "environment.yaml")
    canonical_home = environment["robot"]["home_joint_state"]["joints"]

    extractor_path = ROOT / "scripts" / "extract_scene_urdf_visual_mesh_index.py"
    spec = importlib.util.spec_from_file_location("scene_mesh_index", extractor_path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # Exercise the actual expanded-URDF startup boundary, not a second pose constant.
    controls = ''.join(
        f'<joint name="{name}"><state_interface name="position">'
        f'<param name="initial_value">{value}</param></state_interface></joint>'
        for name, value in canonical_home.items())
    links = '<link name="world"/>'
    parent = 'world'
    for name in JOINT_ORDER:
        links += f'<link name="{name}_link"/><joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{name}_link"/><axis xyz="0 0 1"/></joint>'
        parent = name + '_link'
    _, diagnostics = module.extract_from_urdf(
        f'<robot name="test">{links}<ros2_control name="test" type="system">{controls}</ros2_control></robot>',
        {}, include_diagnostics=True)
    assert diagnostics['initial_joint_source'] == 'ros2_control.initial_value'
    _assert_joint_map_close(diagnostics['ur5_preview_joint_pose']['joints'], canonical_home)


def test_direct_workbench_mount_has_only_mesh_precision_clearance():
    environment = _load_yaml(SCENE / "environment.yaml")
    layout = _load_yaml(SCENE / "layout/workcell_studio_layout.yaml")
    assert 'robot_mount_plate' not in {item['id'] for item in layout['items']}
    assert 'robot_mount_plate' not in {item['id'] for item in environment['assets']}
    table = next(item for item in layout['items'] if item['id'] == 'support_surface_table')
    assert math.isclose(environment['robot']['pose_xyz'][2] - table['surface_z_m'], 1e-5, abs_tol=1e-10)
