from pathlib import Path
import json, subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_scene_urdf_visual_mesh_index_generation():
    script = ROOT / 'scripts' / 'extract_scene_urdf_visual_mesh_index.py'
    proc = subprocess.run(['python3', str(script)], capture_output=True, text=True)
    assert proc.returncode == 0
    assert 'Traceback' not in proc.stdout
    assert 'Traceback' not in proc.stderr

    report = ROOT / 'build' / 'workcell_studio_urdf_visual_mesh_index_report.json'
    assert report.exists()
    data = json.loads(report.read_text())
    assert data['scene_count'] >= 8
    assert data['visual_count'] > 0

    mesh_assets_present = any((ROOT / 'assets').rglob('*.stl')) or any((ROOT / 'assets').rglob('*.dae'))
    if mesh_assets_present:
        assert data['resolved'] > 0

    unresolved_only = True
    ur_scene_resolved = False
    unresolved_warnings = 0

    for scene in (ROOT / 'scenes').iterdir():
        if not scene.is_dir():
            continue
        idx = scene / 'generated' / 'scene_visual_mesh_index.json'
        assert idx.exists()
        payload = json.loads(idx.read_text())
        items = payload.get('visual_items', [])
        assert items
        if any(i.get('resolved') for i in items):
            unresolved_only = False
        unresolved_warnings += sum(1 for i in items if (not i.get('resolved', False)) and i.get('warning'))
        if scene.name.startswith('ur') and any(i.get('resolved', False) and i.get('category') in {'robot_visual', 'gripper_visual', 'environment_visual', 'unknown_visual'} for i in items):
            ur_scene_resolved = True

    assert not unresolved_only
    assert ur_scene_resolved
    assert unresolved_warnings >= 0
