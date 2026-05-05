#!/usr/bin/env python3
import json, subprocess, sys, tempfile
from pathlib import Path

def main() -> int:
    root = Path(__file__).resolve().parents[1]
    script = root / 'scripts' / 'generate_static_sorting_runtime_bridge_payload.py'
    replay = Path(__file__).resolve().parents[3] / 'scripts' / 'replay_emd_bridge_payload.py'

    subprocess.run([sys.executable, str(script)], check=True, capture_output=True, text=True)
    result = subprocess.run([sys.executable, str(script), '--json'], check=True, capture_output=True, text=True)
    payload = json.loads(result.stdout)
    assert payload['schema_version'] == 'emd_grasp_bridge_payload/v1'
    assert payload['ros_interface']['selected'] == 'service'
    assert payload['ros_interface']['service_name'] == 'grasp_requests'
    targets = payload['grasp_task']['grasp_targets']
    assert len(targets) == 3
    ids = {t['object_id'] for t in targets}
    assert ids == {'item_red','item_blue','item_green'}
    dests = {t['destination_id'] for t in targets}
    assert dests == {'bin_a','bin_b','reject_bin'}

    with tempfile.TemporaryDirectory() as td:
        out = Path(td)/'payload.json'
        subprocess.run([sys.executable, str(script), '--output', str(out)], check=True)
        loaded = json.loads(out.read_text())
        assert loaded['schema_version'] == 'emd_grasp_bridge_payload/v1'
        subprocess.run([sys.executable, str(replay), '--payload', str(out), '--scene-package', 'ur5_2f_sorting_test', '--dry-run'], check=True, capture_output=True, text=True)
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
