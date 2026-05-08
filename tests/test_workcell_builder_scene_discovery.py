from pathlib import Path
import re

def _sanitize(name: str) -> str:
    out = ''.join('_' if c.isspace() else c.lower() for c in name)
    out = ''.join(c for c in out if c.isalnum() or c == '_')
    while '__' in out:
        out = out.replace('__','_')
    if out and out[0].isdigit():
        out = 'scene_' + out
    return out

def test_scene_sanitize_example():
    assert _sanitize('My First UR5 Cell') == 'my_first_ur5_cell'

def test_scenes_have_discovery_markers():
    root = Path(__file__).resolve().parents[1] / 'scenes'
    markers = ['package.xml', 'scene_manifest.yaml', 'environment.yaml', 'urdf/scene.urdf.xacro', 'launch/demo.launch.py']
    scene_dirs = [p for p in root.iterdir() if p.is_dir()]
    assert scene_dirs, 'expected scene directories'
    assert any(any((d / m).exists() for m in markers) for d in scene_dirs)
