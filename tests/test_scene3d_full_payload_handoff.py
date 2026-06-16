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
