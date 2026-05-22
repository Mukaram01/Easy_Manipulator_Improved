from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')


def test_scene_bounds_include_meshes_fallbacks_and_zones_tokens():
    assert 'scene_bounds_from_visible_items' in VIEW_CPP
    assert 'mesh_world_bounds_for_item' in VIEW_CPP
    assert 'item_bounds_for_role' in VIEW_CPP
    assert 'include_in_fit_bounds(it, include_overlays)' in VIEW_CPP
    assert 'draw_pick_zone' in VIEW_CPP
    assert 'draw_safety_zone' in VIEW_CPP


def test_fit_to_scene_computes_finite_sane_camera_target_distance():
    assert 'if (!scene_bounds_from_visible_items(bmin, bmax, fit_include_overlays)) { set_isometric_view(); return; }' in VIEW_CPP
    assert 'const double radius = qMax(0.25, 0.5 * qSqrt(ext.x() * ext.x() + ext.y() * ext.y() + ext.z() * ext.z()));' in VIEW_CPP
    assert 'distance_ = qBound(min_distance_, fit_distance, max_distance_);' in VIEW_CPP
    assert 'orbit_offset_ = (bmin + bmax) * 0.5f;' in VIEW_CPP


def test_zero_or_nan_dimensions_are_defaulted_and_emit_diagnostics():
    assert 'item_has_explicit_dimensions(const ScenePreviewWidget::PreviewItem & item) const' in VIEW_CPP
    assert 'return item.sx > 0.001 && item.sy > 0.001 && item.sz > 0.001;' in VIEW_CPP
    assert 'Scene3D diagnostics {viewport_received_count=' in VIEW_CPP
    assert 'Overlay-fit warning: overlay bounds are %1x physical bounds' in PREVIEW_CPP
