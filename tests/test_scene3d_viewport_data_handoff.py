from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
VIEW = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_viewport_data_handoff_and_callbacks_wired():
    for token in ['set_preview_items(filtered_items)', 'preview_item_selected', 'select_cb', 'transform_changed_cb']:
        assert token in PREVIEW or token in MAIN or token in VIEW
