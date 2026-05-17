from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_scene_preview_view_buttons_and_clear_selection_present():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
    for token in ['Isometric', 'Top', 'Front', 'Side', 'Focus Selected', 'Clear Selection']:
        assert token in text


def test_mainwindow_keyboard_nudge_and_units_tooltips_present():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in ['Qt::Key_PageUp', 'Qt::Key_PageDown', 'Nudge step:', 'X position in metres', 'Yaw in radians', 'RoleLocked']:
        assert token in text


def test_focus_selected_centers_on_selected_item():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    assert 'if (it.id != selected_id) continue;' in text
    assert 'orbit_offset_' in text
