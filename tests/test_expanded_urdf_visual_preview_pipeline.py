from pathlib import Path
import json, subprocess
ROOT = Path(__file__).resolve().parents[1]

def test_expanded_pipeline_or_fallback():
    proc = subprocess.run(['python3', str(ROOT/'scripts/extract_scene_urdf_visual_mesh_index.py'), '--scene', 'ur5_2f_test', '--prefer-xacro-expanded'], capture_output=True, text=True)
    assert proc.returncode in (0,3)
    data = json.loads((ROOT/'scenes/ur5_2f_test/generated/scene_visual_mesh_index.json').read_text())
    assert data['extraction_mode'] in ('xacro_expanded','xacro_lite_expanded','best_effort_recursive')
    if data['extraction_mode']=='xacro_expanded':
        assert data['source_expanded_urdf_path'] == 'generated/expanded_scene_preview.urdf'
    if data['extraction_mode']=='xacro_expanded':
        assert '${' not in json.dumps(data.get('visual_items', []))

def test_regen_report_has_new_fields():
    subprocess.run(['python3', str(ROOT/'scripts/regenerate_scene_visual_mesh_indexes.py'), '--scene', 'ur5_2f_test', '--portable'], check=False)
    rpt = json.loads((ROOT/'build/workcell_studio/visual_mesh_index_regeneration_report.json').read_text())
    row = rpt['scenes'][0]
    for k in ['extraction_mode','xacro_available','expanded_urdf_written','safe_for_preview','unresolved_placeholder_count','mesh_backed_count','primitive_fallback_count','stale_index','status']:
        assert k in row

def test_banned_tokens_absent_from_commands_text():
    data = json.loads((ROOT/'scenes/ur5_2f_test/generated/scene_visual_mesh_index.json').read_text())
    txt = ' '.join(data.get('xacro_command', []))
    for banned in ['use_fake_hardware:=false','fake_hardware:=false','ur_robot_driver','ethercat','canopen']:
        assert banned not in txt
