from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_add_to_canvas_creates_real_item_metadata():
    for token in ['new DraggableCanvasItem', 'RoleId', 'RoleDisplayName', 'RoleCategory', 'RoleRole', 'RoleSource', 'RoleSourcePackage', 'RoleWidth', 'RoleDepth', 'RoleHeight', 'item->setSelected(true)', 'undo_stack_.push_back({"add"']:
        assert token in CPP

def test_default_placement_tokens_exist():
    for token in ['default_xy_for_category', 'table', 'conveyor', 'camera', 'pick_zone', 'place_zone', 'QMessageBox::warning']:
        assert token in CPP
