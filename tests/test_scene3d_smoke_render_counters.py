from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')


def test_smoke_json_includes_runtime_render_counter_fields():
    required = [
        'preview_items_count', 'viewport_received_count', 'render_cache_count',
        'visible_count', 'rendered_count', 'skipped_count', 'unique_visible_item_count',
        'mesh_backed_count', 'placeholder_count', 'mesh_rendered_count',
        'generated_fallback_count', 'editable_layout_count', 'primitive_fallback_count',
        'locked_generated_urdf_visual_count', 'labels_drawn', 'labels_suppressed_overlap',
    ]
    for token in required:
        assert f'counters["{token}"]' in MAIN_CPP
