from pathlib import Path


def test_gripper_mount_rpy_default_and_safety_flags_present():
    src = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert '-1.5708 -1.5708 0' in src
    assert 'fake_hardware_first: true' in src
    assert 'runtime_execution_enabled: false' in src
    assert 'motion_command_sent: false' in src
