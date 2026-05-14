from pathlib import Path


def test_templates_ui_contains_required_combos_and_safety():
    text = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
    assert 'UR5 + Robotiq 2F' in text
    assert 'UR5 + suction' in text
    assert 'PREVIEW_ONLY' in text
    assert 'no robot motion commanded' in text
