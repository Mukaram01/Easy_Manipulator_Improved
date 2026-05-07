from __future__ import annotations
import json, shutil, subprocess, sys
from pathlib import Path


def test_perception_bridge_preview_generation_and_validation(tmp_path: Path) -> None:
    scene = tmp_path / 'scene'
    shutil.copytree('scenes/ur5_2f_test', scene)
    out = tmp_path / 'out'
    run = subprocess.run([sys.executable, 'scripts/run_golden_builder_readiness_demo.py', '--scene-package', str(scene), '--output-dir', str(out), '--force', '--json'], capture_output=True, text=True, check=False)
    assert run.returncode == 0
    payload = scene / 'generated' / 'emd_bridge_payload_preview.json'
    report = scene / 'generated' / 'perception_bridge_preview_report.json'
    assert payload.exists() and report.exists()
    v = subprocess.run([sys.executable, 'scripts/validate_perception_bridge_preview.py', str(payload), '--json'], capture_output=True, text=True, check=False)
    assert v.returncode == 0
    p = json.loads(payload.read_text(encoding='utf-8'))
    assert p['source'] == 'perception_replay'
    assert p['task_bridge']['routing_result'] == 'routed'
    assert p['task_bridge']['grasp_strategy']
    assert p['task_bridge']['place_target']
    assert p['task_bridge']['release_strategy']
    assert p['real_hardware_enabled'] is False
    assert p['moveit_plan_service_called'] is False
