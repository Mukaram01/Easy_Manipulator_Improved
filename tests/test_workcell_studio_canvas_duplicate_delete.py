from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_duplicate_and_delete_guards_present():
    assert 'Delete robot is blocked/guarded' in CPP
    assert 'QMessageBox::question(this,"Delete Selected","Delete selected item?")' in CPP
    assert 'RoleLocked' in CPP
