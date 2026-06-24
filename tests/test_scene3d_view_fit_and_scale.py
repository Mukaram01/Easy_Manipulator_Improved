from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
VIEW_CPP = (ROOT/'workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text(encoding='utf-8')


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 0
    for idx in range(brace, len(source)):
        if source[idx] == '{':
            depth += 1
        elif source[idx] == '}':
            depth -= 1
            if depth == 0:
                return source[brace + 1:idx]
    raise AssertionError(f'could not parse function body for {signature}')


def test_scene_fit_uses_visible_items_bounds_and_has_diagnostics():
    assert 'scene_bounds_from_visible_items' in VIEW_CPP
    assert 'include_overlays' in VIEW_CPP
    assert 'Scene3D diagnostics {viewport_received_count=' in VIEW_CPP


def test_default_scene_fit_uses_mixed_physical_scene_bounds_not_generated_mesh_focus():
    fit_scene = _function_body(VIEW_CPP, 'void Scene3DViewportWidget::fit_scene()')

    assert 'scene_bounds_from_visible_items(bmin, bmax, fit_include_overlays)' in fit_scene
    assert 'has_generated_mesh_focus' not in fit_scene
    assert 'last_camera_fit_target_ = QStringLiteral("scene")' in fit_scene

    fit_robot = _function_body(VIEW_CPP, 'void Scene3DViewportWidget::fit_robot()')
    assert 'robot_bounds_from_rendered_visuals(bmin, bmax)' in fit_robot
    assert 'last_camera_fit_target_ = QStringLiteral("robot")' in fit_robot


def test_fit_bounds_include_mesh_urdf_primitive_semantic_fallback_and_layout_items():
    include_fit = _function_body(VIEW_CPP, 'bool include_in_fit_bounds')
    scene_bounds = _function_body(VIEW_CPP, 'bool Scene3DViewportWidget::scene_bounds_from_visible_items')
    item_bounds = _function_body(VIEW_CPP, 'Scene3DViewportWidget::ItemBounds Scene3DViewportWidget::item_bounds_for_role')
    primitive_local = _function_body(VIEW_CPP, 'bool primitive_local_bounds_for_item')
    primitive_world = _function_body(VIEW_CPP, 'bool primitive_world_bounds_for_item')

    assert 'item_has_credible_mesh_handoff(it)' in include_fit
    assert 'item_has_explicit_primitive_dimensions(it)' in include_fit
    assert 'if (helper_overlay) return false;' in include_fit
    assert 'if (generated_urdf_visual) return true;' in include_fit
    assert 'if (it.linked_to_editable_layout_state) return true;' in include_fit
    assert 'source_layer == "mesh_preview"' in include_fit
    assert 'product_physical_role && (mesh_backed || explicit_primitive)' in include_fit

    assert 'include_in_fit_bounds(it, include_overlays)' in scene_bounds
    assert 'item_bounds_for_role(it)' in scene_bounds
    assert 'mesh_world_bounds_for_item(item, mesh_bounds)' in item_bounds
    assert 'primitive_world_bounds_for_item(item, primitive_bounds)' in item_bounds

    assert 'primitive_radius' in primitive_local
    assert 'primitive_length' in primitive_local
    assert 'Semantic fallback/editable boxes are rendered as axis-aligned min-corner boxes.' in primitive_local


def test_product_fit_uses_generous_distance_guards_and_exports_camera_diagnostics():
    fit_product = _function_body(VIEW_CPP, 'void Scene3DViewportWidget::fit_product_view()')
    diagnostics_block = VIEW_CPP.split('SCENE3D_MESH_DIAGNOSTICS_JSON', 1)[1].split('void Scene3DViewportWidget::fit_scene()', 1)[0]

    assert 'fit_include_overlays = false;' in fit_product
    assert 'initial_physical_fit_bounds(bmin, bmax, &ur5_included)' in fit_product
    assert '* 1.22' not in fit_product
    assert 'const double base_fit_distance = product_radius / qTan(fov * 0.5);' in fit_product
    assert 'qMax(qMax(base_fit_distance * 2.4, product_radius * 5.0), 4.0)' in fit_product
    assert 'distance_ = qBound(min_distance_, fit_distance, max_distance_);' in fit_product
    assert 'orbit_offset_ = (bmin + bmax) * 0.5f;' in fit_product
    assert 'qMax(0.06, product_radius * 0.035)' in fit_product

    for token in (
        'root["scene_radius"] = scene_radius_;',
        'root["camera_distance"] = distance_;',
        'root["camera_fit_margin"] = last_camera_fit_margin_;',
        'root["camera_fit_bounds_min"]',
        'root["camera_fit_bounds_max"]',
        'root["camera_fit_bounds_span"]',
    ):
        assert token in diagnostics_block
