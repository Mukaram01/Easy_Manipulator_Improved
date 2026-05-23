from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_counter_handoff_prefers_active_widget_and_tracks_fallback_source():
    assert 'counters["viewport_counter_source"] = QString("active_widget")' in CPP
    assert 'counters["viewport_counter_source"] = QString("missing")' in CPP


def test_candidate_selection_precedence_visible_nonzero_before_hidden_zero():
    assert 'rc.visible_count > 0' in CPP
    assert '(rc.rendered_count > 0 || (rc.render_cache_count > 0 && screenshot_available))' in CPP
