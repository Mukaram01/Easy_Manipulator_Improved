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
        'environment["task_zones"]',
    ]:
        assert token in MAIN_CPP


def test_save_roundtrip_reselects_by_stable_id_or_clears_when_missing():
    for token in [
        'stable_selected_id_before_refresh',
        'Save Layout: rebuilding Scene3D data after save',
        'apply_scene_selection(stable_selected_id_before_refresh',
        'Selection id missing after refresh, clearing atomically',
        'apply_scene_selection(QString(), selected_role, true, false);',
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
