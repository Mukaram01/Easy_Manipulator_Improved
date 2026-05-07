from scripts.render_workcell_builder_metadata import render_metadata


def test_ur5_robotiq_remains_runtime_ready() -> None:
    payload = render_metadata("ur5", "robotiq_2f", "realsense_d435i", "finger_pinch_basic")
    assert payload["selected_capabilities"]["robot"]["capability_id"] == "ur5"
    assert payload["selected_capabilities"]["end_effector"]["capability_id"] == "robotiq_2f_85"
    assert payload["readiness"]["status"] == "fake_hardware_ready"


def test_placeholder_robot_marked_preview_only_warn() -> None:
    payload = render_metadata("generic delta", "suction", "realsense_d435i", "suction_top_basic")
    assert payload["robot"]["preview_only"] is True
    assert payload["robot"]["runtime_supported"] is False
    assert payload["readiness"]["status"] == "preview_only"


def test_generated_metadata_contains_selected_capability_ids() -> None:
    payload = render_metadata("ur5", "robotiq 2f", "intel realsense d435i", "finger_pinch_basic")
    selected = payload["selected_capabilities"]
    assert selected["robot"]["capability_id"] == "ur5"
    assert selected["end_effector"]["capability_id"] == "robotiq_2f_85"
    assert selected["sensor"]["capability_id"] in {"intel_realsense_d435i", "realsense_d435i"}
