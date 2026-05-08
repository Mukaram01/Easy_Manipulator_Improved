from tools.workcell_studio_streamlit import backend


def test_gui_pick_roi_and_filters_export(tmp_path):
    state = backend.default_manual_authoring_state("gui_roi")
    state["selected_assets"] = [
        {"id": "ur5", "role": "robot_base"},
        {"id": "rg2", "role": "end_effector"},
        {"id": "table", "role": "support_surface"},
        {"id": "cam", "role": "camera"},
    ]
    state["pick_sources"][0]["pointcloud_topic"] = "/camera/camera/depth/color/points"
    state["pick_sources"][0]["filters"]["voxel_leaf_size"] = 0.01
    out = backend.export_manual_authoring_bundle(state, tmp_path)
    cell = backend.load_environment_layout(out["cell_definition.yaml"])
    roi = cell["pick_sources"][0]
    assert roi["type"] == "camera_pointcloud_roi"
    assert roi["pointcloud_topic"] == "/camera/camera/depth/color/points"
    assert roi["filters"]["voxel_leaf_size"] == 0.01
