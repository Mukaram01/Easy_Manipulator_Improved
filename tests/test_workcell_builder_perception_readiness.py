from scripts import workcell_builder_gui_workflow as wf

def test_readiness_reports_config_only():
    panel=wf.build_readiness_status_panel({'selected':{},'fake_hardware_default':True})
    assert panel['perception_status'] in {'PERCEPTION_DISABLED','PERCEPTION_READY_CONFIG_ONLY'}
    assert any('Live EPD not launched automatically' in m for m in panel['messages'])
