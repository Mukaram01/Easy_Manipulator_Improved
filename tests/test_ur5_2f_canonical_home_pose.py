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
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -math.pi / 2.0,
    "elbow_joint": math.pi / 2.0,
    "wrist_1_joint": -math.pi / 2.0,
    "wrist_2_joint": -math.pi / 2.0,
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

    # Resolve the scene contract relative to the Xacro itself. This must work
    # both directly from a checkout (Web3D extraction/CI) and from the installed
    # package without requiring `ur5_2f_test` to be discoverable via ament first.
    assert 'name="environment_file" default="$(dirname)/../environment.yaml"' in xacro
    assert '$(find ur5_2f_test)/environment.yaml' not in xacro
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

    # Product View currently uses this deterministic preview posture when the
    # shared UR5 MoveIt initial-position file is all zero. Keep it in parity with
    # the canonical scene home until the extractor consumes scene home directly.
    for name in JOINT_ORDER:
        assert math.isclose(
            float(module.UR5_PREVIEW_HOME_JOINT_POSE[name]),
            float(canonical_home[name]),
            rel_tol=0.0,
            abs_tol=1e-4,
        ), name
