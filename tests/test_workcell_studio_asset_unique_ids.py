from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_unique_id_generation_uses_suffix():
    for token in ['id_prefix_from_category', 'do { new_id = QString("%1_%2")', 'while (exists(new_id))', 'QLatin1Char(\'0\')']:
        assert token in CPP
