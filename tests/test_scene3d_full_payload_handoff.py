from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
VIEWPORT_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
GUI_MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_default_layer_visibility_keeps_generated_mesh_preview_in_full_payload():
    assert 'out.locked_generated_urdf_visual = true;' in CANDIDATE_CPP
    assert 'Product-view defaults should favor authored layout plus generated mesh' in CANDIDATE_CPP
    assert 'Scene3D full payload committed: scene=%1 total=%2 visible=%3 mesh=%4 locked=%5' in MAIN_CPP


def test_viewport_ingest_counters_reflect_combined_editable_and_preview_payload_before_paint():
    ingest_start = VIEWPORT_CPP.index('void Scene3DViewportWidget::ingest_preview_items')
    ingest_end = VIEWPORT_CPP.index('void Scene3DViewportWidget::fit_scene', ingest_start)
    ingest_block = VIEWPORT_CPP[ingest_start:ingest_end]
    for token in [
        'last_render_counters.viewport_received_count = items.size();',
        'last_render_counters.mesh_backed_count = mesh_source_count;',
        'last_render_counters.mesh_rendered_count = 0;',
        'last_render_counters.locked_generated_urdf_visual_count = locked_urdf_count;',
        'last_render_counters.editable_layout_count = editable_layout_count;',
    ]:
        assert token in ingest_block


def test_scene3d_payload_filter_connections_are_null_guarded():
    assert 'if (box) {' in MAIN_CPP
    assert 'connect(box, &QCheckBox::toggled, this, [this](bool) { apply_scene3d_preview_layer_filters(true); });' in MAIN_CPP
    assert 'if (snap_action && snap_to_grid_box_) {' in MAIN_CPP
    assert 'if (fine_move_action && fine_move_mode_box_) {' in MAIN_CPP
    assert 'if (unlock_action && unlock_robot_base_box_) {' in MAIN_CPP
    assert 'if (minimap_action && show_minimap_box_) {' in MAIN_CPP


def test_scene3d_smoke_load_keeps_generated_urdf_layer_additive_for_mesh_index_payloads():
    smoke_start = MAIN_CPP.index('bool MainWindow::load_scene_for_scene3d_smoke')
    smoke_end = MAIN_CPP.index('void MainWindow::open_new_scene_creation_flow', smoke_start)
    smoke_block = MAIN_CPP[smoke_start:smoke_end]

    assert 'apply_scene3d_product_view_layer_defaults_and_commit();' in smoke_block
    assert 'preview_layer_generated_urdf_visual_box_->setChecked(defaults.locked_generated_urdf_visual);' not in smoke_block
    assert 'preview_layer_generated_urdf_visual_box_->setChecked(!has_renderable_editable_mesh_or_primitive);' not in smoke_block
    assert 'has_renderable_editable_mesh_or_primitive' not in smoke_block

    # Regression coverage for the editable-layout + generated mesh-index smoke path:
    # the smoke counters must expose both the assembled static payload and the
    # post-filter counts forwarded through ScenePreviewWidget into the viewport.
    for token in [
        'source_mesh_index_item_count',
        'assembled_preview_item_count',
        'forwarded_to_preview_widget_count',
        'forwarded_to_viewport_count',
        'filtered_visible_candidate_count',
    ]:
        assert token in GUI_MAIN_CPP or token in MAIN_CPP

    # The additive default means generated/locked URDF visuals are eligible in
    # the same layer set as editable rows, mesh previews, and primitive fallbacks.
    assert '"locked_generated_urdf_visual"' in smoke_block
    assert '"editable_layout"' in smoke_block
    assert '"mesh_preview"' in smoke_block
    assert '"primitive_fallback"' in smoke_block


def test_ur5_2f_final_viewport_audit_requires_renderable_ur5_links_table_and_camera():
    audit_start = MAIN_CPP.index('QJsonObject audit_ur5_2f_test_committed_viewport_items')
    audit_end = MAIN_CPP.index('double normalize_angle_radians_with_guard', audit_start)
    audit_block = MAIN_CPP[audit_start:audit_end]

    for link in [
        'base_link_inertia',
        'shoulder_link',
        'upper_arm_link',
        'forearm_link',
        'wrist_1_link',
        'wrist_2_link',
        'wrist_3_link',
    ]:
        assert f'QStringLiteral("{link}")' in audit_block
    assert 'rendered_ur5_link_count' in audit_block
    assert 'rendered_table_count' in audit_block
    assert 'rendered_camera_count' in audit_block
    assert 'missing_required_visible_ur5_links' in audit_block
    assert 'const bool visible = renderable;' in audit_block


def test_loader_filter_warning_is_suppressed_when_required_final_viewport_links_are_visible():
    filter_start = MAIN_CPP.index('if (has_selected_scene() && selected_scene_name() == QStringLiteral("ur5_2f_test"))')
    filter_end = MAIN_CPP.index('const Scene3DTransformParityReadiness transform_parity', filter_start)
    filter_block = MAIN_CPP[filter_start:filter_end]

    assert 'audit_ur5_2f_test_committed_viewport_items(viewport, &missing_required_visible_links)' in filter_block
    assert 'if (!missing_required_visible_links.isEmpty())' in filter_block
    assert 'retained visual rows missing after loader filtering' in filter_block
    assert 'final viewport audit passed for ur5_2f_test required visible UR5 viewport/renderable links' in filter_block


def test_viewport_generated_urdf_renderer_rejects_environment_yaml_semantic_rows_before_mesh_path():
    assert 'not_generated_urdf_renderable_geometry' in VIEWPORT_CPP
    credible_start = VIEWPORT_CPP.index('bool item_has_credible_mesh_handoff')
    credible_end = VIEWPORT_CPP.index('bool is_generated_urdf_visual_item', credible_start)
    credible_block = VIEWPORT_CPP[credible_start:credible_end]
    assert 'path_has_mesh_asset_extension(mesh_path)' in credible_block
    assert '!mesh_path.isEmpty() ||' not in credible_block
    assert 'environment.yaml' in credible_block or 'Authoring files such as environment.yaml' in credible_block


def test_viewport_baked_urdf_transform_branch_applies_baked_pose_and_scale_once():
    transform_start = VIEWPORT_CPP.index('QMatrix4x4 authoritative_world_visual_transform')
    transform_end = VIEWPORT_CPP.index('QMatrix4x4 viewport_world_visual_transform', transform_start)
    transform_block = VIEWPORT_CPP[transform_start:transform_end]
    assert 'if (item.has_baked_world_visual_transform)' in transform_block
    assert 'return transform;' in transform_block.split('if (item.has_baked_world_visual_transform)', 1)[1]
    assert 'visual_origin_applied' not in transform_block.split('if (item.has_baked_world_visual_transform)', 1)[1].split('return transform;', 1)[0]
    final_start = VIEWPORT_CPP.index('QMatrix4x4 final_mesh_transform_matrix')
    final_end = VIEWPORT_CPP.index('void apply_mesh_local_correction_gl', final_start)
    final_block = VIEWPORT_CPP[final_start:final_end]
    assert 'viewport_world_visual_transform(item)' in final_block
    assert 'transform.scale(static_cast<float>(item.mesh_scale_x)' in final_block
    assert 'applied_scale=[' in VIEWPORT_CPP


def test_viewport_deduplicates_identical_generated_camera_visuals():
    assert 'scene3d_camera_dedupe_key' in VIEWPORT_CPP
    assert 'seen_camera_visuals.contains(key)' in VIEWPORT_CPP
    assert 'classify_item_role(*item) == NormalizedRole::Camera' in VIEWPORT_CPP


def test_ur5_audit_excludes_base_link_inertia_as_hard_visible_mesh_requirement():
    audit_start = MAIN_CPP.index('QJsonObject audit_ur5_2f_test_committed_viewport_items')
    audit_end = MAIN_CPP.index('double normalize_angle_radians_with_guard', audit_start)
    audit_block = MAIN_CPP[audit_start:audit_end]
    required_block = audit_block[audit_block.index('const QSet<QString> required_visible_ur5_links'):audit_block.index('const QSet<QString> table_link_tokens')]
    assert 'QStringLiteral("base_link_inertia")' not in required_block
    assert 'excluded_non_visual_ur5_links' in audit_block
    assert 'base_link_inertia: inertial-only/non-visual' in audit_block
