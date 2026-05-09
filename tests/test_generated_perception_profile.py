from scripts import workcell_builder_gui_workflow as wf

def test_profile_defaults_present():
    p=wf.default_perception_profile({})['perception']
    assert p['enabled'] is False
    assert p['camera']['model']=='realsense_d435i'
    assert p['epd']['localization_topic']
    assert p['epd']['tracking_topic']
