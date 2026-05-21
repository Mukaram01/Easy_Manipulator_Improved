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


def test_render_pipeline_structured_diagnostics_non_zero_guards_present():
    for token in [
        'render_cache_count=',
        'rendered_count=',
        'skipped_count=',
        'paintGL cache-only guard: no YAML/file IO in paint path',
    ]:
        assert token in TXT
