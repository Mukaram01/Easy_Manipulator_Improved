from pathlib import Path
import subprocess, json

ROOT = Path(__file__).resolve().parents[1]


def test_checker_has_pass_warn_fail_fields(tmp_path):
    out_json = tmp_path / 'contract.json'
    subprocess.run(['python3', str(ROOT / 'scripts' / 'check_scene3d_canvas_contract.py'), '--scene', 'suction_test', '--json', str(out_json)], check=False)
    payload = json.loads(out_json.read_text(encoding='utf-8'))
    scene = payload['scenes'][0]
    required = ['scene','editable_layout_count','mesh_preview_count','locked_generated_urdf_visual_count','primitive_fallback_count','overlay_count','unsafe_visual_reason_count','unresolved_placeholder_count','contract_status','blockers','suggested_fixes']
    for k in required:
        assert k in scene
    assert scene['contract_status'] in {'PASS','WARN','FAIL'}


def test_no_disallowed_real_hardware_tokens_in_new_contract_assets():
    banned = ['use_fake_hardware:=false','fake_hardware:=false','ur_robot_driver','ethercat','canopen']
    targets = [
        ROOT / 'scripts' / 'check_scene3d_canvas_contract.py',
        ROOT / 'docs' / 'architecture' / 'SCENE3D_CANVAS_CONTRACT.md',
    ]
    for t in targets:
        text = t.read_text(encoding='utf-8').lower()
        for b in banned:
            assert b not in text
