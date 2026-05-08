from tools.workcell_studio_streamlit import backend


def test_gui_invalid_roi_fails_validation():
    state = backend.default_manual_authoring_state("gui_validate")
    state["selected_assets"] = [
        {"id": "ur5", "role": "robot_base"},
        {"id": "rg2", "role": "end_effector"},
        {"id": "table", "role": "support_surface"},
    ]
    state["pick_sources"][0]["crop_box"]["x_min"] = 1.0
    state["pick_sources"][0]["crop_box"]["x_max"] = 0.2
    report = backend.authoring_validation_report(state)
    assert report["status"] == "FAIL"
    assert any(i["code"] == "invalid_crop_box" for i in report["issues"])
