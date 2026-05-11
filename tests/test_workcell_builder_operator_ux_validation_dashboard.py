from pathlib import Path


def _txt(path: str) -> str:
    return Path(path).read_text(encoding='utf-8')


def test_operator_workflow_and_dashboard_labels_exist():
    ui = _txt('workcell_builder/workcell_builder/gui/scene_select.ui')
    for s in [
        'Operator Workflow (Main)',
        '1. Scene',
        '2. Robot + End Effector',
        '3. Objects / Layout',
        '4. Camera / Perception Metadata',
        '5. Task / Grasp Strategy',
        '6. Validate',
        '7. Generate Files',
        'Validation Dashboard',
        'Run Offline Validation',
    ]:
        assert s in ui


def test_golden_demo_not_main_operator_button_and_is_dev_tooling_only():
    ui = _txt('workcell_builder/workcell_builder/gui/scene_select.ui')
    assert 'Create Golden UR5 + Robotiq 2F Cell' in ui
    assert 'Developer Tools:' in ui


def test_validation_statuses_and_generation_gating_strings_present():
    cpp = _txt('workcell_builder/workcell_builder/gui/scene_select.cpp')
    for s in [
        'Scene Schema',
        'Asset Catalog',
        'Robot/Tool Compatibility',
        'Object Placement',
        'Camera Metadata',
        'Task Recipe',
        'Readiness Overlay',
        'Fake-Hardware Smoke',
        'blocker_count',
        'warning_count',
        'Generate Files blocked by readiness blockers',
        'proceeding with warnings (allowed)',
    ]:
        assert s in cpp


def test_no_moveit_execution_or_real_hw_or_streamlit_or_pyyaml_added():
    blob = _txt('workcell_builder/workcell_builder/gui/scene_select.cpp').lower()
    for forbidden in ['getmotionplan', 'execute_trajectory', 'followjointtrajectory', 'pyyaml', 'import yaml']:
        assert forbidden not in blob
