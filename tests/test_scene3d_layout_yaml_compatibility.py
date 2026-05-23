from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_layout_pipeline_mentions_preview_items():
    assert 'preview_items' in CPP
