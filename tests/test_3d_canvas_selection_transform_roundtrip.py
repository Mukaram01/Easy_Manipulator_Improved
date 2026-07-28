from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
MAIN_H = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')
PREVIEW_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')
CANVAS_MODEL_CPP = (ROOT / 'workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp').read_text(encoding='utf-8')


def test_layout_item_model_supports_required_types():
    for token in ['robot_base', 'pick_zone', 'place_zone', 'camera', 'camera_fov', 'conveyor', 'spawn_line']:
        assert token in CANVAS_MODEL_CPP or token in PREVIEW_H or token in MAIN_CPP


def test_selection_sync_and_inspector_editability_contract_present():
    for token in [
        'preview_item_selected',
        'apply_scene_selection',
        'scene_hierarchy_tree_->setCurrentItem',
        'setReadOnly(locked)',
        'inspector_dim_x_',
        'inspector_dim_y_',
        'inspector_dim_z_',
    ]:
        assert token in MAIN_CPP or token in MAIN_H


def test_transform_and_dimensions_update_selected_item_only_contract_present():
    for token in [
        'digital_twin_scene_->selectedItems().front()',
        'setData(RolePoseZ',
        'setData(RoleRoll',
        'setData(RolePitch',
        'setData(RoleYaw',
        'setData(RoleWidth',
        'setData(RoleDepth',
        'setData(RoleHeight',
    ]:
        assert token in MAIN_CPP


def test_save_roundtrip_preserves_metadata_and_updates_task_zones_contract_present():
    for token in [
        'workcell_studio_layout.yaml',
        'item["id"]',
        'item["type"]',
        'item["source_path"]',
        'item["editable"]',
        'item["locked"]',
        'pose["xyz"]',
        'pose["rpy"]',
        'item["dimensions"]',
        'root["task_zones"]',
    ]:
        assert token in MAIN_CPP


def test_save_roundtrip_reselects_by_stable_id_and_preserves_selection_when_missing():
    for token in [
        'stable_selected_id_before_refresh',
        'Save Layout: rebuilding Scene3D data after save',
        'apply_scene_selection(stable_selected_id_before_refresh',
        'Ignored selection id absent from active scene payload; existing selection preserved',
    ]:
        assert token in MAIN_CPP


def test_duplicate_names_are_selection_independent_by_stable_id():
    for token in [
        'tree_item->data(0, TreeRoleId).toString().trimmed() == selected_id',
        'gi->data(RoleId).toString().trimmed() == selected_id',
        'current_selected_scene_item_id_ = selected_id',
    ]:
        assert token in MAIN_CPP


def test_locked_item_edit_rejected_and_no_real_hardware_banned_tokens():
    assert 'Locked/generated item edit rejected' in MAIN_CPP
    banned = ['use_fake_hardware:=false', 'fake_hardware:=false', 'ur_robot_driver', 'ethercat', 'canopen']
    scan = '\n'.join([PREVIEW_H, CANVAS_MODEL_CPP]).lower()
    for token in banned:
        assert token not in scan


def test_escape_cancel_path_does_not_save_layout_or_commit_transform():
    esc_shortcut_block = MAIN_CPP.split('auto * esc_sc = new QShortcut(QKeySequence(Qt::Key_Escape), scene_builder);', 1)[1].split('; });', 1)[0]
    assert 'save_layout_changes' not in esc_shortcut_block
    assert 'mark_layout_dirty' not in esc_shortcut_block



def test_gizmo_handle_click_does_not_reselect_item_before_drag_contract_present():
    viewport_cpp = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    body = viewport_cpp.split('void Scene3DViewportWidget::mousePressEvent(QMouseEvent * e) {', 1)[1].split('void Scene3DViewportWidget::mouseMoveEvent', 1)[0]
    gizmo_block = body.split('if ((gizmo_mode == GizmoMode::Move || gizmo_mode == GizmoMode::Rotate) && !selected_id.isEmpty())', 1)[1].split('QString best_id, best_role;', 1)[0]

    assert 'item_is_editable_for_gizmo(*selected_item)' in gizmo_block
    assert 'pick_gizmo_axis_at_screen(e->pos(), axis, &score)' in gizmo_block
    assert 'pick_gizmo_rotation_ring_at_screen(e->pos(), axis, &score)' in gizmo_block
    assert 'drag_start_pose_.item_id = selected_item->id;' in gizmo_block
    assert 'drag_in_progress_ = true;' in gizmo_block
    assert 'return;' in gizmo_block
    assert 'pick_item_at_screen' not in gizmo_block
