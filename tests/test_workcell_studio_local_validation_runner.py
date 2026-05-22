import json, subprocess
from pathlib import Path

def test_local_validation_outputs_reports(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'out'
    rc = subprocess.run(['python3', str(repo/'scripts/run_workcell_studio_local_validation.py'), '--repo-root', str(repo), '--output-dir', str(out), '--dry-run'], capture_output=True, text=True)
    assert (out / 'workcell_studio_local_validation.json').exists()
    assert (out / 'workcell_studio_local_validation.md').exists()
    payload = json.loads((out / 'workcell_studio_local_validation.json').read_text())
    assert payload['schema'] == 'workcell_studio_local_validation/v1'

def test_local_validation_fail_when_required_missing(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'out2'
    rc = subprocess.run(['python3', str(repo/'scripts/run_workcell_studio_local_validation.py'), '--repo-root', str(repo), '--output-dir', str(out), '--include-gui-smoke', '--require-gui-smoke', '--workspace-root', str(tmp_path/'ws')], capture_output=True, text=True)
    assert rc.returncode != 0


def test_local_validation_consumes_smoke_fail_json_blockers(tmp_path):
    repo = Path(__file__).resolve().parents[1]
    out = tmp_path / 'out3'
    cmd = [
        'python3', str(repo/'scripts/run_workcell_studio_local_validation.py'),
        '--repo-root', str(repo), '--output-dir', str(out), '--include-gui-smoke', '--require-gui-smoke',
        '--workspace-root', str(tmp_path/'missing_ws'), '--executable', '/bin/true', '--scenes', 'ur5_2f_test'
    ]
    rc = subprocess.run(cmd, capture_output=True, text=True)
    assert rc.returncode != 0
    payload = json.loads((out / 'workcell_studio_local_validation.json').read_text(encoding='utf-8'))
    joined = '\n'.join(payload.get('blockers', []))
    assert 'GUI smoke failed for ur5_2f_test' in joined
    assert 'returncode=' in joined
