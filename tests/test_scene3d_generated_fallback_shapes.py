from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text()

def test_generated_fallback_counter_present():
    assert 'generated_fallback_count' in CPP
