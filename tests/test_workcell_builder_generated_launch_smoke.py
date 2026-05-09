from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_humble_launch_template_has_smoke_safe_controller_fallbacks_and_error_hint():
    content = (REPO_ROOT / 'workcell_builder/workcell_builder/templates/ros2/humble/launch/demo.launch.py').read_text(encoding='utf-8')
    assert '_derive_chain_joints_from_urdf' in content
    assert 'URDF-only fallback arm joints for preview/fake hardware' in content
    assert 'Regenerate SRDF with explicit manipulator joint entries or configure robot capability arm_joints.' in content
    assert 'use_fake_hardware' in content
    assert '"allow_trajectory_execution": False' in content


def test_scene_template_keeps_default_scene_and_asset_workspace_paths():
    readme = (REPO_ROOT / 'README.md').read_text(encoding='utf-8')
    assert '~/workcell_ws/src/scenes' in readme
    assert '~/workcell_ws/src/assets' in readme
