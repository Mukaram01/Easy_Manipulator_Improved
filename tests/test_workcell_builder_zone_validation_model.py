from pathlib import Path
CPP = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text()
DASH = Path('workcell_builder/workcell_builder/src_validation_dashboard_model.cpp').read_text()

def test_zone_validator_status_tokens_present():
    for token in ['OK / WARNING / BLOCKED / PREVIEW_ONLY', 'Robot Reach', 'robot_reach', 'camera_roi', 'pick_zone', 'place_zone']:
        assert token in CPP or token in DASH

def test_no_runtime_moveit_calls_for_zone_validation():
    banned = ['move_group', 'MoveIt', 'execute(', 'FollowJointTrajectory']
    section = CPP.split('build_layout_preview_items', 1)[1][:5000]
    assert all(tok not in section for tok in banned)
