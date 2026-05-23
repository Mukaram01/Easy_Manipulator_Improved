from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8')
SMOKE_GUARDS = Path('tests/test_scene3d_transform_editing_guard.py').read_text(encoding='utf-8')
WORKSPACE_GUARDS = Path('tests/test_scene3d_workspace_agnostic_smoke.py').read_text(encoding='utf-8')


def test_smoke_has_visual_quality_blocker_checks():
    assert 'visual_quality_failed' in CPP
    assert "Inspector remains 'No scene selected'" in CPP


def test_no_hardcoded_workspace_paths_in_smoke_related_guard_paths():
    assert '/workspace/' not in CPP
    assert '/home/' not in CPP
    assert 'hardcoded_root_workspace' in WORKSPACE_GUARDS


def test_no_forbidden_real_hardware_launch_tokens_in_guarded_code_paths():
    for forbidden in ['use_fake_hardware:=false', 'fake_hardware:=false', 'ur_robot_driver', 'ethercat', 'canopen']:
        assert forbidden in SMOKE_GUARDS
