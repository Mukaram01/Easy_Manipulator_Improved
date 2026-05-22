from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_visual_mesh_preview_ingestion_logs_nonempty_path():
    assert 'Visual mesh preview items added:' in MAIN
    assert 'Visual mesh index loaded:' in MAIN
    assert 'Skipped visual items:' in MAIN
