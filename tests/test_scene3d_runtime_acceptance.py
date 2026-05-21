import json, subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_runtime_acceptance_script_emits_json_and_markdown(tmp_path):
    out_json = tmp_path / 'scene3d_runtime_acceptance.json'
    out_md = tmp_path / 'scene3d_runtime_acceptance.md'
    subprocess.run(['python3', str(ROOT / 'scripts' / 'validate_scene3d_runtime_acceptance.py'), '--json', str(out_json), '--markdown', str(out_md)], check=True)
    assert out_json.exists() and out_md.exists()
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    assert payload['schema'] == 'workcell_studio_scene3d_runtime_acceptance/v1'
