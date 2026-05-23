from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_preview_not_unavailable_when_rendered_or_received():
    assert 'rendered <= 0' in CPP
    assert 'generated_fallback_count > 0' in CPP
