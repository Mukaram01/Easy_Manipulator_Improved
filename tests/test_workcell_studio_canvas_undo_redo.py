from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')

def test_undo_redo_buttons_wired():
    for token in ['undo_layout_button_', 'redo_layout_button_', 'mark_layout_dirty("Undo")', 'mark_layout_dirty("Redo")']:
        assert token in CPP


def test_scene_builder_undo_redo_have_one_qt_shortcut_owner():
    assert 'undo_action->setShortcut(QKeySequence::Undo)' in CPP
    assert 'redo_action->setShortcut(QKeySequence::Redo)' in CPP
    assert 'new QShortcut(QKeySequence::Undo, scene_builder)' not in CPP
    assert 'new QShortcut(QKeySequence::Redo, scene_builder)' not in CPP


def test_undo_redo_actions_preserve_web3d_routing_and_menu_reuse():
    undo = CPP.split('void MainWindow::undo_layout_edit(){', 1)[1].split(
        'void MainWindow::redo_layout_edit(){', 1
    )[0]
    redo = CPP.split('void MainWindow::redo_layout_edit(){', 1)[1].split(
        'void MainWindow::duplicate_selected_item(){', 1
    )[0]

    assert 'scene_preview_widget_->undo_authoring_edit();' in undo
    assert 'scene_preview_widget_->redo_authoring_edit();' in redo
    assert 'scene_builder_secondary_overflow_menu_->addAction(undo_action);' in CPP
    assert 'scene_builder_secondary_overflow_menu_->addAction(redo_action);' in CPP
