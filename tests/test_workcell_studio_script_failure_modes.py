import json
import subprocess
import sys
from pathlib import Path


def test_layout_merge_missing_scene_reports_blocked_json(tmp_path):
    missing = tmp_path / 'missing_scene'
    script = Path('scripts/workcell_studio_layout_merge.py')
    run = subprocess.run([sys.executable, str(script), str(missing), '--json'], capture_output=True, text=True, check=False)
    assert run.returncode != 0
    payload = json.loads(run.stdout)
    assert payload.get('status') in {'BLOCKED', 'ERROR'}


def test_preview_launch_script_keeps_fake_hardware_guard_tokens():
    text = Path('scripts/workcell_studio_preview_launch.py').read_text(encoding='utf-8')
    assert 'use_fake_hardware:=true' in text
