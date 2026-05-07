from __future__ import annotations
import json
import shutil
import subprocess
import sys
from pathlib import Path


def test_golden_demo_includes_perception_artifacts(tmp_path: Path) -> None:
    scene = tmp_path / 'scene'
    shutil.copytree('scenes/ur5_2f_test', scene)
    out = tmp_path / 'out'
    proc = subprocess.run([sys.executable, 'scripts/run_golden_builder_readiness_demo.py', '--scene-package', str(scene), '--output-dir', str(out), '--force', '--json'], capture_output=True, text=True, check=False)
    assert proc.returncode == 0
    assert (scene / 'generated' / 'perception_profile.yaml').exists()
    assert (scene / 'generated' / 'perception_readiness_report.json').exists()
    manifest = json.loads((out / 'readiness_pack_manifest.json').read_text(encoding='utf-8'))
    assert 'perception' in manifest
    assert manifest['perception']['status'] in ('perception_profile_ready', 'perception_replay_ready', 'perception_partial')
    assert manifest['safety']['real_hardware_enabled'] is False
    assert manifest['safety']['motion_command_sent'] is False
    assert manifest['safety']['runtime_execution_called'] is False
    dashboard = (out / 'readiness_dashboard.html').read_text(encoding='utf-8')
    assert 'Perception readiness' in dashboard
    summary = json.loads((out / 'golden_builder_demo_summary.json').read_text(encoding='utf-8'))
    assert 'perception' in summary
