from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_task_binding_ui_buttons_still_present_and_layout_safety_flags_preserved():
    for token in ['Use Selected as Pick Source', 'Use Selected as Place Target', 'Use Selected as Pick Zone', 'Use Selected as Place Zone', 'Use Selected as Camera', 'fake_hardware_first: true', 'runtime_execution_enabled: false', 'motion_command_sent: false', 'gripper_mount_rpy: [-1.5708, -1.5708, 0]']:
        assert token in CPP
