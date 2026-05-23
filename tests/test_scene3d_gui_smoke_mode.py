from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text()

def test_smoke_has_visual_quality_blocker_checks():
    assert 'visual_quality_failed' in CPP
    assert "Inspector remains 'No scene selected'" in CPP
