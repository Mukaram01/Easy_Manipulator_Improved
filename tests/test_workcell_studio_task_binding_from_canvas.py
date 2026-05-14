from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_task_binding_shortcuts_present():
    for t in ['Use Selected as Pick Source','Use Selected as Place Target','Use Selected as Pick Zone','Use Selected as Place Zone','Use Selected as Camera']:
        assert t in CPP

def test_gripper_mount_and_safety_flags_present():
    for t in ['-1.5708 -1.5708 0','fake_hardware_first: true','runtime_execution_enabled: false','motion_command_sent: false']:
        assert t in Path('workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text(encoding='utf-8') + CPP
