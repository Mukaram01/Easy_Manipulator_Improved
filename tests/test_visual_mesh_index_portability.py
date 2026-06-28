from pathlib import Path
import json, subprocess, yaml
ROOT = Path(__file__).resolve().parents[1]

def test_portable_fields_exist():
    subprocess.run(['python3', str(ROOT/'scripts/extract_scene_urdf_visual_mesh_index.py'), '--scene', 'ur5_2f_test'], check=True)
    p = ROOT/'scenes/ur5_2f_test/generated/scene_visual_mesh_index.json'
    data = json.loads(p.read_text())
    item = data['visual_items'][0]
    assert 'repo_relative_source_path' in item
    assert 'scene_relative_source_path' in item
    assert 'asset_relative_source_path' in item
    if item.get('resolved_source_path','').startswith('/workspace'):
        assert item.get('repo_relative_source_path') or item.get('scene_relative_source_path') or item.get('asset_relative_source_path')

def test_regeneration_report_and_catalog():
    subprocess.run(['python3', str(ROOT/'scripts/regenerate_scene_visual_mesh_indexes.py'), '--scene', 'ur5_2f_test', '--portable'], check=True)
    rpt = json.loads((ROOT/'build/workcell_studio/visual_mesh_index_regeneration_report.json').read_text())
    row = rpt['scenes'][0]
    for k in [
        'scene',
        'status',
        'visual_item_count',
        'mesh_backed_count',
        'primitive_fallback_count',
        'unresolved_count',
        'stale_or_unsafe_count',
        'generated_index_path',
        'postprocess_helper_available',
        'postprocess_changed',
        'postprocess_detail',
        'ur5_runtime_repair_applied',
        'ur5_runtime_repair_added_links',
        'ur5_runtime_repair_added_end_effector_links',
    ]:
        assert k in row
    cat = yaml.safe_load((ROOT/'config/workcell_studio_visual_asset_catalog.yaml').read_text())
    for k in ['robot_ur','gripper_robotiq_2f','gripper_suction','gripper_airpick','table','workbench','conveyor_placeholder','camera_realsense','bin_box_fixture']:
        assert k in cat['categories']

def test_readiness_gate_visual_mesh_index_summary():
    subprocess.run(['python3', str(ROOT/'scripts/run_workcell_studio_scene_readiness_gate.py'), '--dry-run-launches', '--include-visual-assets'], check=False)
    payload = json.loads((ROOT/'build/workcell_studio/workcell_studio_scene_readiness_gate.json').read_text())
    assert 'visual_asset_inventory' in payload
    assert 'visual_mesh_index_regeneration' in payload
