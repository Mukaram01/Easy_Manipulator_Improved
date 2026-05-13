from pathlib import Path

ADD_SCENE_UI = Path('workcell_builder/workcell_builder/gui/addscene.ui').read_text()
SCENE_SELECT_H = Path('workcell_builder/workcell_builder/gui/scene_select.h').read_text()


def test_add_scene_scrollable_grouped_layout():
    assert 'QScrollArea" name="scrollArea"' in ADD_SCENE_UI
    for section in [
        'Scene',
        'Robot',
        'End Effector',
        'Objects / Environment',
        'Work Zones / Metadata Preview',
    ]:
        assert f'<string>{section}</string>' in ADD_SCENE_UI


def test_work_zone_buttons_are_compact_grouped():
    for button in ['add_work_zone', 'edit_work_zone', 'remove_work_zone', 'add_conveyor_flow']:
        assert f'name="{button}"' in ADD_SCENE_UI
    assert 'QGridLayout" name="workzoneButtons"' in ADD_SCENE_UI
    assert 'Delete</string>' not in ADD_SCENE_UI


def test_compact_dialog_actions_and_no_absolute_sections():
    assert 'QDialogButtonBox' in ADD_SCENE_UI
    assert 'name="exit"' in ADD_SCENE_UI
    assert '<layout class="QVBoxLayout" name="dialogLayout"' in ADD_SCENE_UI


def test_no_stale_autoconnect_slot_names():
    assert 'on_refresh_preview_clicked' not in SCENE_SELECT_H
    assert 'on_export_preview_clicked' not in SCENE_SELECT_H
    assert 'on_open_conveyor_sorting_run_console_clicked' not in SCENE_SELECT_H
