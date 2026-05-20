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
    assert data['scene_count'] > 0

    discovered = 0
    unresolved_warnings = 0
    for scene in (ROOT / 'scenes').iterdir():
        if not scene.is_dir():
            continue
        idx = scene / 'generated' / 'scene_visual_mesh_index.json'
        assert idx.exists()
        payload = json.loads(idx.read_text())
        discovered += len(payload.get('visual_items', []))
        unresolved_warnings += sum(1 for i in payload.get('visual_items', []) if (not i.get('resolved', False)) and i.get('warning'))

    assert discovered > 0
    assert unresolved_warnings >= 0
