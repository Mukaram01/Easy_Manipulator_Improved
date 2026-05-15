from pathlib import Path

CPP_MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CPP_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
H_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')


def test_reachability_and_collision_overlay_models_exist():
    for token in [
        'ReachabilityOverlayModel', 'robot_base_id', 'planning_group', 'approximate_reach_min_m',
        'approximate_reach_max_m', 'preferred_work_zone_radius_m', 'pick_source_status', 'place_target_status',
        'CollisionOverlayModel', 'colliding_items', 'near_miss_items', 'metadata_source'
    ]:
        assert token in H_PREVIEW


def test_overlay_menu_tokens_exist():
    for token in ['Reachability Heatmap', 'Collision Warnings', 'Safety Zones', 'Work Envelope', 'Warning Labels']:
        assert token in CPP_PREVIEW


def test_validation_panel_and_warning_tokens_exist():
    for token in [
        'Reachability status', 'Collision status', 'Safety zone status', 'Pick source reach',
        'Place target reach', 'Warning count', 'Preview-only',
        'no robot base found', 'robot reach metadata missing',
        'pick source outside approximate reach', 'place target outside approximate reach',
        'selected item outside approximate reach', 'asset overlap', 'too close to robot base',
        'object below floor/table', 'object intersects safety zone'
    ]:
        assert token in CPP_MAIN or token in CPP_PREVIEW


def test_2d_fallback_and_safety_runtime_boundaries_preserved():
    assert '2D Layout' in CPP_PREVIEW
    forbidden = ['move_group', 'MoveGroupInterface', 'FollowJointTrajectory', 'controller_manager/switch_controller', 'publish(']
    for token in forbidden:
        assert token not in CPP_MAIN


def test_regression_tokens_absent():
    assert 'QPolygonF{' not in CPP_PREVIEW
    assert 'select_preview_item(item->' not in CPP_MAIN
