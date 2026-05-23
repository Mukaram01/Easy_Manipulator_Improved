from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_handoff_fail_token_updated():
    assert 'active_viewport_counter_handoff_failed' in CPP


def test_paint_completion_fallback_logic_tracks_rendered_or_cache_with_screenshot():
    assert 'counters.rendered_count > 0' in CPP
    assert '(counters.render_cache_count > 0 && screenshot_saved)' in CPP
    assert 'paint_cycle_completed' in CPP
