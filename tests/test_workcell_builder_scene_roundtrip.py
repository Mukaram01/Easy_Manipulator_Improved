from pathlib import Path

ROUNDTRIP_CPP = Path('workcell_builder/workcell_builder/src_workcell_scene_roundtrip.cpp').read_text(encoding='utf-8')
SCENE_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')


def test_roundtrip_status_validator_and_scene_version_markers_exist():
    for token in ['validate_roundtrip_scene_state', 'roundtrip_status_label', 'workcell_scene/v1', 'Legacy/partial']:
        assert token in ROUNDTRIP_CPP


def test_edit_save_path_keeps_backup_and_safety_metadata_defaults_present():
    for token in ['environment.yaml.bak', 'YAML parse failure', 'fake_hardware_first', 'runtime_execution_enabled', 'selected_template_']:
        assert token in SCENE_CPP


def test_gripper_mount_orientation_regression_marker_present_in_repo():
    test_cpp = Path('tests/test_workcell_builder_gripper_mount_rpy.py').read_text(encoding='utf-8')
    assert '-1.5708 -1.5708 0' in test_cpp
