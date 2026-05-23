from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')

def test_preview_unavailable_guard_for_rendered_items():
    assert 'preview_status_untruthful' in CPP
    assert 'header_preview_status' in CPP
    assert 'workflow_preview_status' in CPP
