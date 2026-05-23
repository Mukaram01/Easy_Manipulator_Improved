from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text()

def test_smoke_counter_handoff_blocker_token():
    assert 'smoke_counter_handoff_failed' in CPP
    assert 'processEvents' in CPP
