from scripts import workcell_builder_gui_workflow as wf


def test_validation_action_backend_headless():
    state={"selected":{},"fake_hardware_default":True}
    report=wf.validate_manual_cell_state(state)
    assert report["status"]=="FAIL"
    assert any(i["code"]=="missing_robot" for i in report["issues"])


def test_validation_warn_preview_only():
    state={"selected":{"robot":"ur5","tool":"2f","support_surface":"table","pick_area":"bin","place_target":"tray","grasp_strategy":"top"},"preview_only_assets":["x"]}
    report=wf.validate_manual_cell_state(state)
    assert report["status"]=="WARN"
