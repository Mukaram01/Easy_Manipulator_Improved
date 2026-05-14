from pathlib import Path

MAIN_CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_preview_gate_warns_on_stale_layout_merge():
    assert 'Layout changed since last generation. Run Generate Scene / Layout Merge before preview.' in MAIN_CPP
