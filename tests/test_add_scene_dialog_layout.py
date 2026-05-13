from pathlib import Path

ADD_SCENE_UI = Path('workcell_builder/workcell_builder/gui/addscene.ui').read_text()
ADD_SCENE_H = Path('workcell_builder/workcell_builder/gui/addscene.h').read_text()
ADD_SCENE_CPP = Path('workcell_builder/workcell_builder/gui/addscene.cpp').read_text()


def test_add_scene_scrollable_grouped_layout():
    assert 'QScrollArea" name="scrollArea"' in ADD_SCENE_UI
    assert '<property name="widgetResizable"><bool>true</bool></property>' in ADD_SCENE_UI
    for section in ['Scene', 'Robot', 'End Effector', 'Objects / Environment', 'Work Zones / Metadata Preview']:
        assert f'<string>{section}</string>' in ADD_SCENE_UI


def test_compact_dialog_actions_and_no_stale_exit_button():
    assert 'QDialogButtonBox' in ADD_SCENE_UI
    assert 'name="exit"' not in ADD_SCENE_UI
    assert '<connections/>' in ADD_SCENE_UI


def test_no_stale_on_ok_autoconnect_slot():
    assert 'on_ok_clicked' not in ADD_SCENE_H
    assert 'on_ok_clicked' not in ADD_SCENE_CPP
    assert 'connect(ui->actionButtonBox, &QDialogButtonBox::accepted' in ADD_SCENE_CPP
    assert 'connect(ui->actionButtonBox, &QDialogButtonBox::rejected' in ADD_SCENE_CPP
