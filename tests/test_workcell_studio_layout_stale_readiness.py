from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/src_workcell_studio_layout_editor.cpp').read_text(encoding='utf-8')

def test_layout_stale_readiness_labels_present():
    assert 'Layout changed since last acceptance' in CPP
    assert 'Run acceptance again' in CPP
