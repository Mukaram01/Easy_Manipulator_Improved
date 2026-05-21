from pathlib import Path
import json, subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_contract_document_exists_and_contains_required_rule():
    text = (ROOT / 'docs' / 'architecture' / 'SCENE3D_CANVAS_CONTRACT.md').read_text(encoding='utf-8')
    for token in ['editable_layout','mesh_preview','locked_generated_urdf_visual','primitive_fallback','overlay']:
        assert token in text
    assert 'must not remove primitive fallback, mesh preview, xacro-expanded preview, or refresh behavior' in text


def test_contract_checker_generates_json_and_markdown(tmp_path):
    out_json = tmp_path / 'report.json'
    out_md = tmp_path / 'report.md'
    proc = subprocess.run(['python3', str(ROOT / 'scripts' / 'check_scene3d_canvas_contract.py'), '--scene', 'suction_test', '--json', str(out_json), '--markdown', str(out_md)], capture_output=True, text=True)
    assert out_json.exists()
    assert out_md.exists()
    data = json.loads(out_json.read_text(encoding='utf-8'))
    assert 'overall_status' in data
    assert data['scenes']
