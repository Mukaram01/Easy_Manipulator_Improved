from tools.workcell_studio_streamlit import backend


def test_missing_pick_roi_and_required_ingredients_fail():
    state = backend.default_manual_authoring_state("validate")
    state["pick_sources"] = []
    report = backend.authoring_validation_report(state)
    assert report["status"] == "FAIL"
    assert any(i["code"] == "missing_pick_area" for i in report["issues"])


def test_preview_only_asset_warn_and_runtime_blocked():
    state = backend.default_manual_authoring_state("validate2")
    state["selected_assets"] = [
        {"id": "r", "role": "robot_base"},
        {"id": "g", "role": "gripper"},
        {"id": "t", "role": "support_surface"},
        {"id": "x", "role": "fixture", "preview_only": True},
    ]
    report = backend.authoring_validation_report(state)
    assert report["status"] in {"WARN", "OK"}
    assert report["runtime_ready"] is False
