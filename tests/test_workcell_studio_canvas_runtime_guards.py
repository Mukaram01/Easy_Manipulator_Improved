from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_canvas_and_layout_guard_strings_present():
    for needle in [
        'No scene selected',
        'Layout has unsaved edits. Save Layout first.',
        'No saved layout found (layout/workcell_studio_layout.yaml).',
    ]:
        assert needle in MAIN
