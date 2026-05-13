from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_next_action_status_present():
    assert 'Next recommended action' in CPP
