from tools.workcell_studio_streamlit import backend


def test_gui_placement_role_collision_export(tmp_path):
    state = backend.default_manual_authoring_state("gui_place")
    state["selected_assets"] = [
        {"id": "ur5", "role": "robot_base"},
        {"id": "rg2", "role": "end_effector"},
        {"id": "table", "role": "support_surface"},
        {"id": "cam", "role": "camera"},
    ]
    state["placements"] = [
        {
            "id": "bin_a",
            "asset_ref": "bin_standard",
            "role": "bin",
            "collision_mode": "bounding_box",
            "pose": {"frame": "world", "xyz": [0.6, -0.2, 0.8], "rpy": [0.0, 0.0, 0.0]},
            "scale": 1.0,
        }
    ]
    out = backend.export_manual_authoring_bundle(state, tmp_path)
    env = backend.load_environment_layout(out["environment_layout.yaml"])
    assert env["assets"][0]["role"] == "bin"
    assert env["assets"][0]["collision_mode"] == "bounding_box"
