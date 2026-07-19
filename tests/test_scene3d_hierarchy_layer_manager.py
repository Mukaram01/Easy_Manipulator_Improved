from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
PREVIEW_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def mainwindow_function(name: str) -> str:
    start = MAIN_CPP.index(f'void MainWindow::{name}')
    next_void = MAIN_CPP.find('\nvoid MainWindow::', start + 1)
    return MAIN_CPP[start:] if next_void == -1 else MAIN_CPP[start:next_void]


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


def test_hierarchy_exposes_stable_id_and_detection_metadata_roles():
    for token in ['TreeRoleStableId', 'TreeRoleCameraId', 'TreeRoleFrameId', 'TreeRoleDetectionLabel', 'TreeRoleConfidence', 'TreeRoleTrackingId', 'TreeRoleSnapshotSourceFile', 'TreeRoleAlignmentWarning']:
        assert token in MAIN_CPP



def test_scene_hierarchy_compact_object_rows_and_single_layers_section():
    assert 'scene_hierarchy_tree_->setHeaderLabels({"Name", "Type", "State"});' in MAIN_CPP
    assert 'scene_hierarchy_tree_->setTextElideMode(Qt::ElideRight);' in MAIN_CPP
    assert 'scene_hierarchy_tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);' in MAIN_CPP
    assert 'header()->setSectionResizeMode(0, QHeaderView::Stretch);' in MAIN_CPP
    assert 'header()->setSectionResizeMode(1, QHeaderView::Interactive);' in MAIN_CPP
    assert 'scene_hierarchy_tree_->setColumnWidth(1, 120);' in MAIN_CPP
    assert 'scene_hierarchy_tree_->setColumnWidth(2, 72);' in MAIN_CPP

    hierarchy_setup = MAIN_CPP[MAIN_CPP.index('hierarchy_layout->addWidget(new QLabel("<b>Scene Hierarchy</b>"));'):MAIN_CPP.index('scene_tab_layout->addWidget(hierarchy_card);')]
    assert 'new QGroupBox("Layers", hierarchy_card)' in hierarchy_setup
    assert hierarchy_setup.count('new QCheckBox(') == 6

    populate = mainwindow_function('populate_scene_hierarchy()')
    add_node = populate[populate.index('auto add_tree_node = [&]'):populate.index('auto include_preview_item_in_hierarchy')]
    assert 'new QTreeWidgetItem(scene_hierarchy_tree_' in add_node
    assert 'new QTreeWidgetItem(parent' not in add_node
    assert 'ensure_group' not in populate
    assert 'Warnings / Missing Assets' not in populate
    assert 'helper_file' not in populate
    assert 'detail_tooltip' in add_node
    assert 'node->setToolTip(0, detail_tooltip);' in add_node
    assert 'node->setToolTip(1' in add_node
    assert 'node->setToolTip(2, detail_tooltip);' in add_node


def test_hierarchy_selection_layer_state_and_empty_state_contract():
    populate = mainwindow_function('populate_scene_hierarchy()')
    assert 'include_preview_item_for_scene3d(p, enabled_layers)' in populate
    assert 'preview_layer_editable_layout_box_->isChecked()' in populate
    assert 'preview_layer_generated_urdf_visual_box_->isChecked()' in populate
    assert 'box->setChecked(true);' in MAIN_CPP
    assert 'set_checked_blocked(preview_layer_editable_layout_box_, defaults.editable_layout);' in MAIN_CPP
    assert 'connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked' in MAIN_CPP
    assert 'apply_scene_selection(selected_id, selected_role, false, true);' in MAIN_CPP
    assert 'apply_scene_selection(selected_id, selected_role, false, false);' in MAIN_CPP
    assert 'No scene items' in populate
    assert 'apply_scene_selection(QString(), QStringLiteral("unknown"), true, false);' in populate
    assert 'empty->setFlags(empty->flags() & ~Qt::ItemIsSelectable);' in populate
