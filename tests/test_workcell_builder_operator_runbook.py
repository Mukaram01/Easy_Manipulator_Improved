from pathlib import Path


def test_operator_runbook_contains_required_workflow_commands_and_safety():
    text = Path('docs/manuals/WORKCELL_BUILDER_OPERATOR_WORKFLOW.md').read_text(encoding='utf-8')
    for token in [
        'Next Steps',
        'Copy Build Command',
        'Copy Launch Command',
        'colcon build --symlink-install --packages-select <scene>',
        'ros2 launch <scene> demo.launch.py use_fake_hardware:=true',
        'Fake hardware remains default',
    ]:
        assert token in text
