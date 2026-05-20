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


def test_locked_urdf_preview_inspector_and_transform_controls_are_read_only():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'Reason: locked/preview-only',
        'setReadOnly(locked)',
        'inspector_apply_button_->setEnabled(!locked)',
        'if (i->data(RoleLocked).toBool()) return;',
    ]:
        assert token in text


def test_save_layout_skips_locked_preview_items_and_logs_paired_summary_counts():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    for token in [
        'if (gi->data(RoleLocked).toBool()) continue;',
        'Editable layout: %1 items | URDF visual preview: %2 locked items',
        "has 0 editable layout items and 0 URDF visual preview locked items",
    ]:
        assert token in text


def test_focus_selected_centers_on_selected_item():
    text = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    assert 'if (it.id != selected_id) continue;' in text
    assert 'orbit_offset_' in text


def test_info_chip_and_viewport_have_locked_urdf_and_separated_overlay_mesh_counters():
    viewport = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    preview = (ROOT / 'workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
    assert 'Items %1 • Mesh %2 • Boxes %3 • Missing %4' in viewport
    assert 'Overlays %1 • Locked URDF %2' in viewport
    assert 'Items %1 M%2 B%3 Miss%4 Ov%5 L-URDF%6' in preview
def test_default_label_suppression_and_metadata_tag_storage_tokens_present():
    main_text = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
    view_text = (ROOT / 'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')
    assert 'p.metadata_tags = tag;' in main_text
    assert 'if (!selected && is_urdf_visual && missing_reason.isEmpty()' in view_text
