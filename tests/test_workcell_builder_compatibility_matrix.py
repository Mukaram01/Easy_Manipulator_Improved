from scripts.workcell_builder_compatibility_matrix import CompatibilityStatus, evaluate_compatibility
from scripts.render_workcell_builder_metadata import render_metadata


def test_supported_ur5_robotiq_baseline_fake_hw() -> None:
    result = evaluate_compatibility(
        robot="UR5",
        gripper_tool="Robotiq 2F",
        task_template="pick_place",
        grasp_strategy="finger_pinch_basic",
        hardware_mode="fake",
    )
    assert result.status == CompatibilityStatus.FAKE_HARDWARE_ONLY


def test_placeholder_robot_preview_only() -> None:
    result = evaluate_compatibility(
        robot="Fanuc placeholder",
        gripper_tool="suction",
        task_template="pick_place",
        grasp_strategy="suction_top_basic",
    )
    assert result.status == CompatibilityStatus.PREVIEW_ONLY


def test_invalid_tool_grasp_combinations() -> None:
    result = evaluate_compatibility(
        robot="UR5",
        gripper_tool="suction",
        task_template="pick_place",
        grasp_strategy="finger_pinch_basic",
    )
    assert result.status == CompatibilityStatus.INVALID


def test_real_hardware_mode_produces_guarded_warning() -> None:
    result = evaluate_compatibility(
        robot="UR5",
        gripper_tool="suction",
        task_template="sorting",
        grasp_strategy="suction_top_basic",
        hardware_mode="real",
    )
    assert result.status == CompatibilityStatus.WARN
    assert any("Real hardware mode" in r for r in result.reasons)


def test_custom_stl_unknown_collision_warns() -> None:
    result = evaluate_compatibility(
        robot="UR5",
        gripper_tool="Robotiq 2F",
        task_template="sorting",
        grasp_strategy="finger_pinch_basic",
        custom_stl_assets=[{"path": "foo.stl", "collision_known": False}],
    )
    assert result.status in (CompatibilityStatus.WARN, CompatibilityStatus.FAKE_HARDWARE_ONLY)
    assert any("unknown collision" in r.lower() for r in result.reasons)


def test_generated_metadata_includes_compatibility_result() -> None:
    payload = render_metadata(
        "UR5", "Robotiq 2F", "RealSense D435i", "finger_pinch_basic", task_template="pick_place"
    )
    assert payload["compatibility"]["status"] == "FAKE_HARDWARE_ONLY"
    assert payload["compatibility"]["reasons"]
