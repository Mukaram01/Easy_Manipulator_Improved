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
