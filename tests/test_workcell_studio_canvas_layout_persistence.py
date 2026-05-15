from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_add_to_canvas_persists_environment_layout_yaml():
    assert 'Add to Canvas' in CPP
    assert 'save_layout_changes();' in CPP
    assert 'environment_layout.yaml' in CPP


def test_pose_and_preview_only_fields_written():
    for token in ['pose["x"]', 'pose["y"]', 'pose["z"]', 'pose["roll"]', 'pose["pitch"]', 'pose["yaw"]', 'preview_only']:
        assert token in CPP


def test_remove_action_is_layout_instance_only_and_non_destructive():
    assert 'Remove Selected Layout Item' in CPP
    assert 'Remove selected layout instance from environment_layout.yaml?' in CPP
    assert 'fs::remove(' not in CPP


def test_malformed_yaml_backup_marker_present():
    assert '.malformed_backup_' in CPP
    assert 'Malformed environment_layout.yaml backup failed. Not overwriting.' in CPP


def test_ui_labels_present():
    for label in ['Save Layout', 'Remove Selected Layout Item', 'Validate Layout']:
        assert label in CPP


def test_no_runtime_motion_command_introduced():
    assert 'run_grasp_execution' not in CPP
