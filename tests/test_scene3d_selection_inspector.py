from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_inspector_exposes_transform_editor_controls_and_units():
    for token in ['inspector_x_', 'inspector_y_', 'inspector_z_', 'inspector_roll_', 'inspector_pitch_', 'inspector_yaw_',
                  'inspector_dim_x_', 'inspector_dim_y_', 'inspector_dim_z_', 'inspector_apply_button_', 'inspector_revert_button_']:
        assert token in MAIN_CPP
    assert 'X position in metres' in MAIN_CPP
    assert 'Roll in radians' in MAIN_CPP


def test_selection_refresh_restore_and_clear_logs_present():
    assert 'Preview selection restored after refresh:' in PREVIEW_CPP
    assert 'Preview selection cleared after refresh (id missing):' in PREVIEW_CPP
    assert 'Selection id missing after refresh, clearing atomically:' in MAIN_CPP
    assert "Save Layout: no selected stable item id to reselect." in MAIN_CPP


def test_selection_reselects_when_id_exists_after_refresh():
    assert 'apply_scene_selection(stable_selected_id_before_refresh' in MAIN_CPP


def test_visual_layers_remain_selectable_and_distinct():
    contract = (ROOT / 'docs/architecture/SCENE3D_CANVAS_CONTRACT.md').read_text(encoding='utf-8')
    for layer in ['editable_layout', 'mesh_preview', 'locked_generated_urdf_visual', 'primitive_fallback', 'overlay']:
        assert layer in contract


def test_locked_items_remain_read_only_in_inspector():
    assert 'inspector_apply_button_->setEnabled(!locked)' in MAIN_CPP
    assert 'inspector_revert_button_->setEnabled(!locked)' in MAIN_CPP
    assert 'Locked/generated item edit rejected' in MAIN_CPP


def test_inspector_prioritizes_authoritative_transform_controls():
    assert 'connect_button(inspector_apply_button_, &MainWindow::apply_selection_transform_from_editor)' in MAIN_CPP
    assert 'connect_button(inspector_revert_button_, &MainWindow::revert_selection_transform_editor)' in MAIN_CPP
    assert 'setObjectName("sceneBuilderInspectorAdvancedDetails")' in MAIN_CPP
    assert 'advanced_details_group->setChecked(false)' in MAIN_CPP
    assert 'advanced_details_contents->setVisible(false)' in MAIN_CPP
    assert 'No item selected' in MAIN_CPP
    assert 'sb->clear(); sb->setEnabled(false); sb->setReadOnly(true)' in MAIN_CPP
    assert 'inspector_apply_button_->setEnabled(false)' in MAIN_CPP
    assert 'Locked preview item' in MAIN_CPP
    assert 'inspector_apply_button_->setEnabled(!locked)' in MAIN_CPP
    assert 'Selected item source: %1' not in MAIN_CPP
    assert MAIN_CPP.count('Selected item source_path: %1') == 1
    for token in ['Read-only details:', 'detection_label:', 'snapshot_source_file:', 'locked_reason:', 'camera_id:', 'frame_id:', 'confidence:', 'Selected item ID: %1', 'Selected item active_visual_source: %1']:
        assert token in MAIN_CPP
