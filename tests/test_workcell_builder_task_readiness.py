from scripts import workcell_builder_gui_workflow as wf


def test_placeholder_templates_preview_only_not_runtime_ready():
    s={"selected":{"robot":"ur5","tool":"robotiq_2f"},"fake_hardware_default":True}
    c=wf.ensure_task_grasp_config(s)
    c["task"]["template"]="sorting_placeholder"
    c["task"]["pick"]["source_ref"]="a"
    c["task"]["place"]["target_ref"]="b"
    v=wf.validate_manual_cell_state(s)
    panel=wf.build_readiness_status_panel(s,v)
    assert panel["badges"]["PREVIEW_ONLY"] is True
