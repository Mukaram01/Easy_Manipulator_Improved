import json, subprocess
from pathlib import Path


def test_local_validation_consumes_wrapper_fail_json(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'out'
    cmd = [
        'python3', str(repo/'scripts/run_workcell_studio_local_validation.py'),
        '--repo-root', str(repo), '--output-dir', str(out), '--include-gui-smoke', '--require-gui-smoke',
        '--workspace-root', str(tmp_path/'missing_ws'), '--executable', '/bin/false', '--scenes', 'ur5_2f_test'
    ]
    rc = subprocess.run(cmd, capture_output=True, text=True)
    assert rc.returncode != 0
    payload = json.loads((out / 'workcell_studio_local_validation.json').read_text(encoding='utf-8'))
    joined = '\n'.join(payload.get('blockers', []))
    assert 'gui_smoke[ur5_2f_test] blocker: app_smoke_json_missing' in joined
    assert 'child_command:' in joined
