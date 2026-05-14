from pathlib import Path

SCRIPTS = [
    "scripts/workcell_studio_layout_merge.py",
    "scripts/validate_workcell_studio_generated_scene.py",
    "scripts/workcell_studio_demo_mode.py",
    "scripts/workcell_studio_preview_launch.py",
    "scripts/run_workcell_studio_golden_flow.py",
]


def test_runtime_helper_scripts_exist_from_repo_root():
    for script in SCRIPTS:
        assert Path(script).is_file(), script
