from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PREVIEW_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
VIEWPORT_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
VIEWPORT_H = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.h').read_text(encoding='utf-8')


def test_preview_widget_delegates_render_counters_to_viewport():
    assert 'viewport->render_debug_counters()' in PREVIEW_CPP
    assert 'out.last_paint_completed = counters.last_paint_completed;' in PREVIEW_CPP


def test_setting_preview_items_updates_viewport_handoff_counters_before_paint():
    assert 'void Scene3DViewportWidget::ingest_preview_items' in VIEWPORT_CPP
    for token in ['viewport_received_count = items.size()', 'visible_count = visible_item_count', 'render_cache_count = mesh_cache_.size()']:
        assert token in VIEWPORT_CPP


def test_viewport_debug_counters_include_paint_marker():
    assert 'bool last_paint_completed{ false };' in VIEWPORT_H
    assert 'last_render_counters.last_paint_completed = true;' in VIEWPORT_CPP
