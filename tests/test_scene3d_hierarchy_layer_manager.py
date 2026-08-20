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
    hierarchy_setup = MAIN_CPP[MAIN_CPP.index('auto * hierarchy_title = new QLabel("<b>Scene Hierarchy</b>"'):MAIN_CPP.index('left_vertical_splitter->addWidget(hierarchy_card);')]
    assert 'new QGroupBox("Layers", hierarchy_card)' in hierarchy_setup
    assert hierarchy_setup.count('new QCheckBox(') == 6
    assert 'sceneHierarchySearch' in hierarchy_setup
    assert 'setContextMenuPolicy(Qt::CustomContextMenu)' in hierarchy_setup
    refresh = mainwindow_function('refresh_scene_hierarchy_tree_from_current_items()')
    assert 'new QTreeWidgetItem(scene_hierarchy_tree_' in refresh
    for group in ['Robot', 'Tool', 'Environment', 'Perception', 'Task']:
        assert group in refresh
    assert 'detail_tooltip' in refresh
    assert 'node->setToolTip(0, detail_tooltip);' in refresh
    assert 'header->setSectionResizeMode(0, QHeaderView::Stretch);' in refresh


def test_hierarchy_selection_layer_state_and_empty_state_contract():
    refresh = mainwindow_function('refresh_scene_hierarchy_tree_from_current_items()')
    assert 'is_user_facing_scene_hierarchy_item(p)' in refresh
    assert 'const QSignalBlocker hierarchy_signals(scene_hierarchy_tree_);' in refresh
    assert 'connect(scene_hierarchy_tree_, &QTreeWidget::itemClicked' in MAIN_CPP
    assert 'No scene items' in refresh
    assert 'empty->setFlags(empty->flags() & ~Qt::ItemIsSelectable);' in refresh

def test_user_facing_hierarchy_policy_covers_synthetic_preview_contract():
    policy_start = MAIN_CPP.index('bool is_user_facing_scene_hierarchy_item(')
    policy_end = MAIN_CPP.index('\nQJsonObject scene3d_viewport_pose_json', policy_start)
    policy = MAIN_CPP[policy_start:policy_end]

    # The policy must use stable provenance/identity fields rather than making
    # renderer visibility or a friendly display name authoritative.
    for field in [
        'source_layer', 'role', 'category', 'editable', 'locked',
        'active_visual_source', 'visual_index_link', 'metadata_tags',
    ]:
        assert f'item.{field}' in policy
    assert 'display_name' not in policy
    assert 'include_preview_item_for_scene3d' not in policy

    visible_synthetic_records = {
        'object_01': ('object', 'editable_layout'),
        'object_02': ('object', 'editable_layout'),
        'support_surface_table': ('support_surface/table', 'editable_layout'),
        'realsense_overhead': ('camera', 'editable_layout'),
        'target_bin_default': ('place target/bin', 'editable_layout'),
        'canonical_robot': ('robot', 'locked_generated_urdf_visual'),
        'canonical_tool': ('end_effector/tool', 'locked_generated_urdf_visual'),
    }
    for stable_id, (role, layer) in visible_synthetic_records.items():
        assert 'canonical_robot_or_tool' in policy or role not in {'robot', 'end_effector/tool'}
        assert stable_id.startswith(('object_', 'support_', 'realsense_', 'target_', 'canonical_'))
        assert layer in {'editable_layout', 'locked_generated_urdf_visual'}

    # Synthetic generated/diagnostic identities map to explicit policy
    # exclusions even when their renderer layers are enabled.
    assert 'generated_visual_identity' in policy
    assert 'visual_index_link.trimmed().isEmpty()' in policy
    for token in ['warning', 'helper', 'keepout', 'exclusion', 'commissioning', 'drop_zone']:
        assert f'QStringLiteral("{token}")' in policy
    hidden_synthetic_ids = [
        'generated_urdf::camera_link::visual_0',
        'robot_reach',
        'warning_anchor',
        'safety_helper',
        'commissioning_pose',
        'drop_zone_helper',
        'generated_urdf::shoulder_link::visual_0',
    ]
    assert all(hidden_synthetic_ids)
