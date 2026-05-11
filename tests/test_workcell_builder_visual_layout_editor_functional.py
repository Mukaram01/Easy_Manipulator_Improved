from pathlib import Path


def test_open_visual_layout_editor_action_is_wired_in_object_placement_manager():
    txt = Path('workcell_builder/workcell_builder/gui/object_placement_dialog.cpp').read_text(encoding='utf-8')
    for needle in ['Open Visual Layout Editor', 'EnvironmentLayoutEditor editor', 'editor.set_objects(model_.objects())']:
        assert needle in txt


def test_canvas_items_are_selectable_draggable_and_sync_markers_exist():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    for needle in [
        'QGraphicsRectItem', 'ItemIsMovable', 'ItemIsSelectable', 'update_model_from_item_move',
        'snap_to_grid_', 'Grid Size', 'world_metres_to_canvas_pixels', 'canvas_pixels_to_world_metres'
    ]:
        assert needle in txt


def test_pose_validation_and_warnings_and_missing_mesh_markers_exist():
    txt = Path('workcell_builder/workcell_builder/gui/environment_layout_editor.cpp').read_text(encoding='utf-8')
    for needle in ['reject NaN/inf values', 'suspiciously large positions', 'missing mesh warning']:
        assert needle in txt


def test_yaml_roundtrip_helpers_exist_without_pyyaml_dependency():
    hdr = Path('workcell_builder/workcell_builder/include/object_placement_model.hpp').read_text(encoding='utf-8')
    src = Path('workcell_builder/workcell_builder/src_object_placement_model.cpp').read_text(encoding='utf-8')
    for needle in ['serialize_placed_objects_to_environment_yaml', 'parse_placed_objects_from_environment_yaml', 'save_environment_layout', 'load_environment_layout']:
        assert needle in hdr and needle in src
    assert 'PyYAML' not in src and 'python3-yaml' not in src


def test_summary_preview_readiness_markers_and_alias_policy_untouched():
    scene = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    assert 'visual_layout_editor_used' in scene
    fix = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in fix and 'ensure_workspace_alias "scenes"' in fix
