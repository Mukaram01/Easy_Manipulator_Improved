from scripts import workcell_builder_gui_workflow as wf

def test_launch_template_safety_default():
    cmd=wf.copy_fake_hardware_launch_command({})['message']
    assert 'use_fake_hardware:=true' in cmd
