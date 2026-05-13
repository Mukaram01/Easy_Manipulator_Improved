from pathlib import Path

GUI_CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')


def test_opening_existing_scene_refreshes_visual_canvas_preview_items():
    for token in ['build_layout_preview_items', 'visual_layout_canvas', 'selectionChanged']:
        assert token in GUI_CPP


def test_partial_scene_preview_warnings_and_safety_badges_present():
    for token in ['No scene selected. Create or open a scene', 'Safety/Home', 'fake_hardware_first | no_runtime_motion']:
        assert token in GUI_CPP
