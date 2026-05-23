from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_paint_completion_fallback_logic_is_encoded_for_smoke_readiness():
    assert '(rc.rendered_count > 0 || (rc.render_cache_count > 0 && screenshot_available))' in CPP
    assert 'paint_cycle_not_completed: direct_accessor_rendered_count=0 render_cache_count=0 screenshot_saved=true' in CPP


def test_smoke_counter_handoff_failure_token_is_present():
    assert 'active_viewport_counter_handoff_failed' in CPP
