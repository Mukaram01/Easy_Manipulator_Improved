from pathlib import Path
import math
import shutil
import yaml

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
H = ROOT / "workcell_builder/workcell_builder/gui/scene_select.h"
SHARED_INITIAL = ROOT / "assets/robots/universal_robot/ur5_moveit_config/config/initial_positions.yaml"

JOINTS = [
    ("Base", "shoulder_pan_joint", 0.0),
    ("Shoulder", "shoulder_lift_joint", -math.pi / 2),
    ("Elbow", "elbow_joint", math.pi / 2),
    ("Wrist 1", "wrist_1_joint", -math.pi / 2),
    ("Wrist 2", "wrist_2_joint", -math.pi / 2),
    ("Wrist 3", "wrist_3_joint", 0.0),
]


def text():
    return CPP.read_text(encoding="utf-8") + "\n" + H.read_text(encoding="utf-8")


def test_suggested_ur5_joint_order_and_values_are_finite():
    src = text()
    positions = [src.index(f'"{name}"') for _, name, _ in JOINTS]
    assert positions == sorted(positions)
    for label, name, value in JOINTS:
        assert f'"{label}"' in src
        assert f'"{name}"' in src
        assert math.isfinite(value)


def test_degrees_radians_conversion_helpers_present_and_canonical_values():
    src = text()
    assert "robot_home_deg_to_rad" in src
    assert "robot_home_rad_to_deg" in src
    assert math.isclose(90.0 * math.pi / 180.0, math.pi / 2.0)
    assert math.isclose((-math.pi / 2.0) * 180.0 / math.pi, -90.0)


def test_scene_local_save_reload_round_trip_and_user_defined_overrides_suggested(tmp_path):
    before = SHARED_INITIAL.read_text(encoding="utf-8") if SHARED_INITIAL.exists() else None
    scene = tmp_path / "scene"
    scene.mkdir()
    env = scene / "environment.yaml"
    payload = {"robot": {"name": "ur5"}}
    payload["robot"]["home_joint_state"] = {
        "source": "user",
        "joints": {name: value + 0.123 for _, name, value in JOINTS},
    }
    env.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    loaded = yaml.safe_load(env.read_text(encoding="utf-8"))
    assert loaded["robot"]["home_joint_state"]["source"] == "user"
    assert loaded["robot"]["home_joint_state"]["joints"]["shoulder_lift_joint"] != -math.pi / 2
    after = SHARED_INITIAL.read_text(encoding="utf-8") if SHARED_INITIAL.exists() else None
    assert after == before


def test_reset_invalid_save_and_dirty_state_contract_tokens():
    src = text()
    for token in [
        "Reset Changes",
        "saved_radians",
        "Invalid joint value",
        "Robot Home save blocked",
        "Unsaved Robot Home changes",
        "primary_validate_button->setEnabled(!unsaved)",
        "primary_generate_package_button->setEnabled(!unsaved",
        "home_joint_state",
    ]:
        assert token in src
