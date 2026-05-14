from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

def test_drag_move_handlers_present():
    assert 'ItemIsMovable' in CPP
    assert 'selectionChanged' in CPP
    assert 'Snap to Grid' in CPP
    assert 'RoleId' in CPP

def test_undo_redo_duplicate_delete_hooks_present():
    for needle in ['undo_layout_edit', 'redo_layout_edit', 'duplicate_selected_item', 'delete_selected_item']:
        assert needle in CPP
