from pathlib import Path

def test_self_test_flag_exists():
    src = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
    assert '--self-test-gui' in src
    assert '--gui-acceptance-check' in src
    assert '/tmp/workcell_builder_gui_acceptance_report.json' in src


def test_visual_layout_canvas_controls_exist():
    ui = Path('workcell_builder/workcell_builder/gui/scene_select.ui').read_text(encoding='utf-8')
    for name in [
        'visual_layout_canvas_group',
        'current_cell_assets_group',
        'fit_cell_action',
        'toggle_grid_action',
        'toggle_roi_action',
        'export_layout_preview_action',
        'fake_hardware_default_label',
    ]:
        assert f'name="{name}"' in ui
    assert 'EPD' not in ui
