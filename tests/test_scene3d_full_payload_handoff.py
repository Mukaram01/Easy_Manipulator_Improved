from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_candidate_assembly.cpp').read_text(encoding='utf-8')
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
VIEWPORT_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_default_layer_visibility_keeps_generated_mesh_preview_in_full_payload():
    assert 'out.locked_generated_urdf_visual = true;' in CANDIDATE_CPP
    assert 'Generated/locked URDF visuals are part of the full Scene3D payload' in CANDIDATE_CPP
    assert 'Scene3D full payload committed: scene=%1 editable=%2 preview=%3 total=%4 visible=%5 mesh=%6 locked=%7' in MAIN_CPP


def test_viewport_ingest_counters_reflect_combined_editable_and_preview_payload_before_paint():
    ingest_start = VIEWPORT_CPP.index('void Scene3DViewportWidget::ingest_preview_items')
    ingest_end = VIEWPORT_CPP.index('void Scene3DViewportWidget::fit_scene', ingest_start)
    ingest_block = VIEWPORT_CPP[ingest_start:ingest_end]
    for token in [
        'last_render_counters.viewport_received_count = items.size();',
        'last_render_counters.mesh_backed_count = mesh_backed_count;',
        'last_render_counters.mesh_rendered_count = mesh_backed_count;',
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
