from pathlib import Path

CPP_MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
CPP_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.cpp').read_text(encoding='utf-8')
H_PREVIEW = Path('workcell_builder/workcell_builder/gui/scene_preview_widget.h').read_text(encoding='utf-8')


def test_overlay_model_fields_exist():
    for token in [
        'TaskOverlayModel', 'task_type', 'pick_source_id', 'place_target_id', 'reject_target_id',
        'grasp_strategy', 'approach_axis', 'approach_distance', 'retreat_axis', 'retreat_distance',
        'object_class', 'warnings'
    ]:
        assert token in H_PREVIEW


def test_pick_place_reject_route_and_ar_tokens_exist():
    for token in [
        'pick source zone',
        'place target zone',
        'reject target zone',
        'Task Route',
        'Approach/Retreat',
        'grasp=',
    ]:
        assert token in CPP_PREVIEW


def test_overlay_menu_and_status_tokens_exist():
    for token in [
        'Task Route',
        'Pick/Place Zones',
        'Approach/Retreat',
        'Labels',
        'Task type:',
        'Pick source:',
        'Place target:',
        'Strategy/ref:',
    ]:
        assert token in CPP_MAIN or token in CPP_PREVIEW


def test_overlay_warning_tokens_exist():
    for token in [
        'missing pick source',
        'missing place target',
        'unknown approach axis',
        'unknown grasp strategy',
        'pick/place overlap',
        'Task overlay unavailable: missing task intent',
    ]:
        assert token in CPP_MAIN or token in CPP_PREVIEW


def test_2d_fallback_and_safety_constraints_unchanged():
    assert '2D Layout' in CPP_PREVIEW
    forbidden = ['cmd_vel', 'trajectory_msgs', 'FollowJointTrajectory', 'publish(']
    for token in forbidden:
        assert token not in CPP_MAIN


def test_regressions_absent():
    assert 'QPolygonF{' not in CPP_PREVIEW
    assert 'select_preview_item(item->' not in CPP_MAIN
