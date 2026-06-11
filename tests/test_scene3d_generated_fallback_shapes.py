from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp').read_text()


def test_generated_fallback_counter_present():
    assert 'generated_fallback_count' in CPP


def test_semantic_role_tokens_are_classified():
    expected_tokens = [
        'support_surface',
        'work_surface',
        'place_zone',
        'place_target',
        'target_bin',
        'conveyor',
        'belt',
        'camera',
        'realsense',
        'safety_zone',
        'keepout',
        'home_pose',
        'safety_pose',
    ]
    for token in expected_tokens:
        assert token in CPP


def test_clean_semantic_roles_have_render_branches():
    expected_roles = [
        'NormalizedRole::Table',
        'NormalizedRole::PlaceBin',
        'NormalizedRole::Conveyor',
        'NormalizedRole::PickZone',
        'NormalizedRole::PlaceZone',
        'NormalizedRole::Camera',
        'NormalizedRole::SafetyZone',
        'NormalizedRole::HomePose',
    ]
    clean_role_block = CPP[CPP.index('bool is_clean_semantic_primitive_role'):CPP.index('bool should_suppress_missing_geometry_marker_for_semantic_role')]
    draw_branch = CPP[CPP.index('bool Scene3DViewportWidget::draw_clean_semantic_primitive'):CPP.index('void Scene3DViewportWidget::draw_box')]
    for role in expected_roles:
        assert role in clean_role_block
        assert role in draw_branch


def test_distinct_semantic_render_helpers_exist():
    expected_helpers = [
        'draw_table_slab(draw_item)',
        'draw_place_target_bin(draw_item)',
        'draw_conveyor(draw_item)',
        'draw_pick_zone(draw_item)',
        'draw_place_zone(draw_item)',
        'draw_camera_body_with_frustum(draw_item)',
        'draw_safety_zone(draw_item)',
        'draw_home_pose_marker(draw_item)',
    ]
    for helper in expected_helpers:
        assert helper in CPP


def test_unknown_geometry_diagnostic_path_is_preserved():
    assert 'REJECT_MISSING_GEOMETRY: no mesh metadata or explicit primitive dimensions' in CPP
    assert 'draw_missing_geometry_marker(it)' in CPP
    assert 'Unknown physical items continue through the missing-geometry path below.' in CPP
