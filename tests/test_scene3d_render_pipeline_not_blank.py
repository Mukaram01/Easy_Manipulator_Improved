from pathlib import Path

TXT = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def test_render_pipeline_not_blank_contract_tokens_present():
    for token in [
        'draw_ground_grid_pass();',
        'draw_world_axes_pass();',
        'primitive_fallback',
        'draw_truthful_item_geometry',
        'Scene3D runtime render: received=',
    ]:
        assert token in TXT
