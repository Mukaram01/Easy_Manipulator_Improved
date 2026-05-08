from tools.workcell_studio_streamlit import backend


def test_pick_area_camera_pointcloud_roi_serializes(tmp_path):
    state = backend.default_manual_authoring_state("cell_roi")
    out = backend.export_manual_authoring_bundle(state, tmp_path)
    assert "cell_definition.yaml" in out
    payload = backend.load_environment_layout(tmp_path / "cell_definition.yaml")
    assert payload["pick_sources"][0]["type"] == "camera_pointcloud_roi"
    crop = payload["pick_sources"][0]["crop_box"]
    assert crop["x_min"] < crop["x_max"] and crop["y_min"] < crop["y_max"] and crop["z_min"] < crop["z_max"]


def test_invalid_crop_box_fails_validation():
    roi = backend.default_pick_roi()
    roi["crop_box"]["x_min"] = 1.0
    roi["crop_box"]["x_max"] = 0.1
    issues = backend.validate_pick_roi(roi)
    assert any(i["code"] == "invalid_crop_box" for i in issues)
