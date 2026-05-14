from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_undo_redo_buttons_wired():
    for token in ['undo_layout_button_', 'redo_layout_button_', 'mark_layout_dirty("Undo")', 'mark_layout_dirty("Redo")']:
        assert token in CPP
