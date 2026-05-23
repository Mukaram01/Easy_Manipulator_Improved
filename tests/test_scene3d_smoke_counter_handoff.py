from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')

def test_handoff_fail_token_updated():
    assert 'active_viewport_counter_handoff_failed' in CPP
