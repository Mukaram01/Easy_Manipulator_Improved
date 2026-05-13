from pathlib import Path

def test_layout_canvas_controls_and_tokens_present():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    cpp = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for token in ['visual_layout_canvas','fit_cell_action','reset_view_action','toggle_grid_action','toggle_reach_action','toggle_roi_action','export_layout_preview_action']:
        assert token in ui
    for token in ['build_layout_preview_items','robot_base','conveyor_1','inspection_target','Safety/Home','selectionChanged']:
        assert token in cpp
