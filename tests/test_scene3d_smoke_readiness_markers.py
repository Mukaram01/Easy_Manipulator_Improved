from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_readiness_markers_present_and_item_optional():
    for token in [
        'readiness_markers', 'hierarchy_ready', 'inspector_ready', 'log_ready',
        'screenshot_ready', 'render_ready', 'selected_scene_ready', 'selected_item_required'
    ]:
        assert token in MAIN_CPP
    assert 'no_item_selected_by_default' in MAIN_CPP


def test_no_generic_timeout_message_when_markers_false():
    assert 'Scene3D readiness failed' in MAIN_CPP
    assert 'viewport_received_count=%1 visible_count=%2 rendered_count=%3 render_cache_count=%4 skipped_count=%5' in MAIN_CPP


def test_hierarchy_readiness_uses_child_row_count_consistently():
    assert 'const bool hierarchy_ready = (tree != nullptr && hierarchy_child_rows > 0);' in MAIN_CPP
