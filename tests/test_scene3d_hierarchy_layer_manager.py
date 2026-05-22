from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
PREVIEW_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_required_hierarchy_groups_and_layer_tokens_exist():
    required_runtime_groups = [
        'editable_layout',
        'mesh_preview',
        'primitive_fallback',
        'camera',
        'robot',
        'overlay',
        'warning',
        'missing',
    ]
    for token in required_runtime_groups:
        assert token in MAIN_CPP or token in VIEW_CPP

    assert ('generated_urdf_visual' in MAIN_CPP) or ('locked_generated_urdf_visual' in MAIN_CPP)


def test_row_metadata_contract_tokens_exist():
    for token in ['id', 'source_layer', 'active_visual_source', 'editable', 'linked_to_editable_layout_state']:
        assert token in PREVIEW_H

    assert 'generated_preview' in MAIN_CPP or 'generated_urdf_visual' in MAIN_CPP
    assert 'primitive_fallback' in MAIN_CPP and 'mesh_preview' in MAIN_CPP
    assert 'missing geometry' in VIEW_CPP or 'mesh missing on disk' in VIEW_CPP


def test_selection_sync_contract_tokens_exist():
    assert 'on_hierarchy_item_selected' in MAIN_CPP
    assert 'apply_scene_selection(selected_id, selected_role, false, true);' in MAIN_CPP
    assert 'on_canvas_selection_changed' in MAIN_CPP
    assert 'apply_scene_selection(selected_id, selected_role, false, false);' in MAIN_CPP
    assert 'Selection id missing after refresh, clearing atomically:' in MAIN_CPP
    assert 'Preview selection cleared after refresh (id missing):' in PREVIEW_CPP
    assert 'apply_scene_selection(stable_selected_id_before_refresh' in MAIN_CPP
    assert 'scene_hierarchy_tree_' in MAIN_CPP and 'selected_id' in MAIN_CPP


def test_generated_locked_items_are_read_only_and_drag_guarded():
    assert 'inspector_apply_button_->setEnabled(!locked)' in MAIN_CPP
    assert 'inspector_revert_button_->setEnabled(!locked)' in MAIN_CPP
    assert 'Locked/generated item edit rejected' in MAIN_CPP
    assert "generated_robot_visual" in VIEW_CPP
    assert 'return it.editable && it.linked_to_editable_layout_state;' in VIEW_CPP


def test_visibility_toggles_are_preview_only_and_avoid_generated_write_paths():
    assert 'Mesh preview mode is visual-only and does not alter generated runtime files.' in PREVIEW_CPP
    assert 'toggle_warnings_box_' in MAIN_CPP
    assert 'toggle_labels_box_' in MAIN_CPP

    toggle_block_start = MAIN_CPP.index('connect(toggle_grid_box_')
    toggle_block_end = MAIN_CPP.index('connect(scene_hierarchy_tree_', toggle_block_start)
    toggle_block = MAIN_CPP[toggle_block_start:toggle_block_end]

    guarded_paths = [
        'environment.yaml',
        'layout/workcell_studio_layout.yaml',
        'generated/scene_visual_mesh_index.json',
        'generated/scene.urdf.xacro',
        'generated/expanded_scene_preview.urdf',
    ]
    for path in guarded_paths:
        assert path not in toggle_block

    assert 'Scene3D diagnostics {model_items_count=' in MAIN_CPP
    assert 'Scene3D diagnostics {model_items_count=' in MAIN_CPP
    assert 'Scene3D diagnostics {preview_items_count=' in PREVIEW_CPP


def test_hierarchy_exposes_stable_id_and_detection_metadata_roles():
    for token in ['TreeRoleStableId', 'TreeRoleCameraId', 'TreeRoleFrameId', 'TreeRoleDetectionLabel', 'TreeRoleConfidence', 'TreeRoleTrackingId', 'TreeRoleSnapshotSourceFile', 'TreeRoleAlignmentWarning']:
        assert token in MAIN_CPP
