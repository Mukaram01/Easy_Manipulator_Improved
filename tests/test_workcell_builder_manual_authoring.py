from tools.workcell_studio_streamlit import backend


def test_manual_authoring_state_without_ros():
    state = backend.default_manual_authoring_state("cell_a")
    assert state["mode"] == backend.MANUAL_AUTHORING_MODE
    assert state["fake_hardware_default"] is True


def test_selection_and_placement_serialization(tmp_path):
    state = backend.default_manual_authoring_state("cell_b")
    state["selected_assets"] = [
        {"id": "robot", "role": "robot_base", "display_name": "UR5"},
        {"id": "tool", "role": "gripper", "display_name": "2F-85"},
    ]
    state["placements"] = [{"id": "table", "asset_ref": "table_standard", "pose": {"frame": "world", "xyz": [0.5, 0, 0], "rpy": [0, 0, 0]}}]
    out = backend.export_manual_authoring_bundle(state, tmp_path)
    assert (tmp_path / "selected_assets.json").exists()
    assert out["environment_layout.yaml"].endswith("environment_layout.yaml")
