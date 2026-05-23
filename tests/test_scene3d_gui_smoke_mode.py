from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
CANVAS_CONTRACT = Path('scripts/check_scene3d_canvas_contract.py').read_text(encoding='utf-8')
RUNTIME_ACCEPTANCE = Path('scripts/validate_scene3d_runtime_acceptance.py').read_text(encoding='utf-8')


def test_scene3d_smoke_mode_contains_required_json_counter_handoff_keys():
    for token in ['viewport_candidates', 'active_viewport_candidate_index', 'viewport_counter_source']:
        assert token in CPP


def test_updated_smoke_paths_avoid_hardcoded_workspace_and_forbidden_real_hardware_tokens():
    smoke_scope = '\n'.join([CPP, CANVAS_CONTRACT, RUNTIME_ACCEPTANCE]).lower()
    assert '/home/' not in smoke_scope
    banned = [
        'use_fake_hardware:=false',
        'fake_hardware:=false',
        'ur_robot_driver',
        'ethercat',
        'canopen',
        'realsense2_camera',
        'easy_perception_deployment',
    ]
    for token in banned:
        assert token not in smoke_scope
