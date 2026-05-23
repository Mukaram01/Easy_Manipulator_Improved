from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_overlay_wireframe_tokens_present():
    assert 'DashLine' in CPP
    assert 'camera FOV wedge/cone' in CPP
