from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()


def test_placeholder_controls_are_disabled_with_reason():
    assert 'Disabled: this control requires feature-complete editor integration and is intentionally blocked.' in CPP
