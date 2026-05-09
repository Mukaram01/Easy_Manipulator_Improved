import json
from pathlib import Path

def test_preview_markers_fixture_shape():
    path = Path('scripts/workcell_builder_gui_workflow.py')
    txt = path.read_text(encoding='utf-8')
    assert 'preview_markers.json' in txt
    assert 'task_flow' in txt
