from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')


def test_asset_catalog_add_to_3d_and_double_click_tokens():
    for token in [
        'Add to Canvas',
        'itemDoubleClicked',
        'Place Asset Mode: Add to 3D Canvas',
    ]:
        assert token in CPP


def test_metadata_warning_and_safety_tokens_present():
    for token in [
        'missing dimensions/source metadata',
        'overlap',
        'outside workspace',
        'too close to robot base',
        'below floor/table',
        'locked item',
    ]:
        assert token in CPP


def test_scene_hierarchy_and_inspector_sync_tokens_present():
    for token in [
        'populate_scene_hierarchy();',
        'select_preview_item(selected_id)',
        'xyz=(',
        'Source:',
        'RoleId',
    ]:
        assert token in CPP


def test_save_layout_backup_token_present_and_2d_fallback_retained():
    assert 'Backup before write created' in CPP
    assert '2D Layout' in PREVIEW


def test_no_runtime_motion_publish_tokens_introduced():
    forbidden = ['/joint_trajectory_controller/joint_trajectory', 'follow_joint_trajectory', 'controller_manager/switch_controller']
    for token in forbidden:
        assert token not in CPP
