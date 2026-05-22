from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = (ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
SMOKE_PY = (ROOT / 'scripts/run_workcell_builder_scene3d_gui_smoke.py').read_text(encoding='utf-8')


def test_smoke_fails_for_placeholder_only_hierarchy_and_unselected_scene_conditions():
    assert 'preview_runtime_ready' in MAIN_CPP
    assert 'preview_visible_count' in MAIN_CPP
    assert 'for (int i = 0; i < scene_hierarchy_tree_->topLevelItemCount(); ++i) {' in MAIN_CPP
    assert 'if (!has_selected_scene()) return {};' in MAIN_CPP


def test_smoke_json_fields_include_runtime_visibility_mesh_fallback_hierarchy_scene_and_screenshot():
    required_fields = [
        'preview_visible_count',
        'scene3d_mesh_count',
        'scene3d_fallback_count',
        'selected_scene_state_',
        'scene_hierarchy_tree_',
    ]
    for field in required_fields:
        assert field in MAIN_CPP

    assert 'artifacts = {"json": str(args.output), "screenshot": str(args.screenshot) if args.screenshot else None}' in SMOKE_PY
    assert 'for field in ["viewport_received_count", "render_cache_count", "rendered_count", "hierarchy_rows_count", "selectable_count"]:' in SMOKE_PY
