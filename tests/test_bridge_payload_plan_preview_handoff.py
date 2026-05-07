from __future__ import annotations
import json, subprocess, sys
from pathlib import Path


def test_generate_and_validate_handoff(tmp_path: Path) -> None:
    scene = Path('scenes/ur5_2f_test').resolve()
    bridge = scene / 'generated' / 'emd_bridge_payload_preview.json'
    report = scene / 'generated' / 'perception_bridge_preview_report.json'
    task = scene / 'generated' / 'workcell_builder_task_intent.yaml'

    subprocess.run([sys.executable, 'scripts/generate_perception_bridge_preview.py', '--perception-profile', str(scene / 'generated' / 'perception_profile.yaml'), '--detected-objects', 'tests/fixtures/perception/detected_objects_snapshot_golden.yaml', '--task-intent', str(task), '--environment-layout', str(scene / 'generated' / 'environment_layout.yaml'), '--output-payload', str(bridge), '--output-report', str(report), '--json'], check=False)

    rviz = tmp_path / 'rviz_moveit_plan_preview_session.json'
    rviz.write_text(json.dumps({'rviz_moveit': {'suggested_launch': {'command': 'ros2 launch ur5_2f_test demo.launch.py use_fake_hardware:=true launch_rviz:=true'}}}), encoding='utf-8')
    handoff = tmp_path / 'bridge_payload_plan_preview_handoff.json'
    gen = subprocess.run([sys.executable, 'scripts/generate_bridge_payload_plan_preview_handoff.py', '--bridge-payload', str(bridge), '--rviz-session', str(rviz), '--scene-package', str(scene), '--output', str(handoff), '--json'], capture_output=True, text=True, check=False)
    assert gen.returncode in (0, 1)
    assert handoff.exists()
    payload = json.loads(handoff.read_text(encoding='utf-8'))
    assert payload['source_bridge_payload_path'].endswith('generated/emd_bridge_payload_preview.json')
    assert payload['safety_flags']['real_hardware_enabled'] is False
    assert payload['safety_flags']['motion_command_sent'] is False
    assert payload['safety_flags']['runtime_execution_called'] is False
    assert payload['safety_flags']['moveit_plan_service_called'] is False

    val = subprocess.run([sys.executable, 'scripts/validate_bridge_payload_plan_preview_handoff.py', str(handoff), '--json'], capture_output=True, text=True, check=False)
    assert val.returncode == 0
