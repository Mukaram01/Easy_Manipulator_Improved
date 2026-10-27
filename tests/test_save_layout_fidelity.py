"""Save Layout fidelity: no unrelated camera rewrites, no wholesale YAML reformatting."""
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

from workcell_studio_layout_merge import merge  # noqa: E402

STALE_MIRROR = [0.4, -0.25, 0.85, 0, 1.57, 0]


def camera_scene(tmp_path, camera_xyz):
    scene = tmp_path / 'scene'
    (scene / 'layout').mkdir(parents=True)
    asset = {'id': 'cam', 'role': 'camera', 'frame': 'world', 'pose_xyz': [0.4, -0.36, 0.85], 'pose_rpy': [0, 1.57, 0]}
    bin_asset = {'id': 'bin', 'role': 'target_bin', 'frame': 'world', 'pose_xyz': [0.45, 0.22, 0.1], 'pose_rpy': [0, 0, 0]}
    env = {'environment': {'assets': [asset, bin_asset]},
           'camera': {'camera_id': 'cam', 'pose': list(STALE_MIRROR),
                      'calibration_status': 'authored_extrinsics_require_physical_verification'}}
    (scene / 'environment.yaml').write_text(yaml.safe_dump(env, sort_keys=False))
    items = [{'id': 'cam', 'role': 'camera', 'pose': {'xyz': list(camera_xyz), 'rpy': [0, 1.57, 0]}},
             {'id': 'bin', 'role': 'target_bin', 'pose': {'xyz': [0.56, 0.3, 0.1], 'rpy': [0, 0, 0]}}]
    (scene / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': items}, sort_keys=False))
    return scene


def test_unrelated_save_leaves_a_stale_camera_mirror_alone_and_warns(tmp_path):
    scene = camera_scene(tmp_path, [0.4, -0.36, 0.85])  # only the bin moved
    report = merge(scene, save_authored=True)
    env = yaml.safe_load((scene / 'environment.yaml').read_text())
    assert env['camera']['pose'] == STALE_MIRROR
    assert env['environment']['assets'][1]['pose_xyz'] == [0.56, 0.3, 0.1]
    assert any('camera cam' in warning and 'left unchanged' in warning for warning in report['merge_warnings'])


def test_moving_the_camera_moves_its_mirror(tmp_path):
    scene = camera_scene(tmp_path, [0.5, -0.36, 0.85])
    merge(scene, save_authored=True)
    env = yaml.safe_load((scene / 'environment.yaml').read_text())
    assert env['camera']['pose'] == [0.5, -0.36, 0.85, 0, 1.57, 0]


HEAD_STYLE = """schema_version: workcell_studio_layout/v1
metadata:
  canonical_layout_path: layout/workcell_studio_layout.yaml
items:
- id: support_surface_table
  display_name: Industrial Workbench
  role: support_surface
  pose:
    xyz:
    - 0.7106668243408203
    - -0.2
    - -0.0008999994993209839
- id: target_bin_default
  display_name: Destination Bin
  role: target_bin
  pose:
    xyz:
    - 0.45
    - 0.22
    - 0.1
"""

# What yaml-cpp emits for the same document after moving only the bin.
YAML_CPP_STAGED = """schema_version: workcell_studio_layout/v1
metadata:
  canonical_layout_path: layout/workcell_studio_layout.yaml
items:
  - id: support_surface_table
    display_name: !<tag:yaml.org,2002:str> Industrial Workbench
    role: support_surface
    pose:
      xyz:
        - 0.71066682434082029
        - -0.20000000000000001
        - -0.00089999949932098393
  - id: target_bin_default
    display_name: !<tag:yaml.org,2002:str> Destination Bin
    role: target_bin
    pose:
      xyz:
        - 0.56000000000000005
        - 0.30230645000000001
        - 0.10000000000000001
"""


def test_native_save_keeps_authored_formatting_and_exact_numbers(tmp_path):
    target = tmp_path / 'workcell_studio_layout.yaml'
    target.write_text(HEAD_STYLE)
    staged = tmp_path / 'staged.yaml'
    staged.write_text(YAML_CPP_STAGED)
    subprocess.run([sys.executable, str(ROOT / 'scripts/authored_yaml.py'), 'rewrite', str(target), str(staged)], check=True)
    written = target.read_text()
    assert 'tag:yaml.org' not in written and '0.71066682434082029' not in written
    changed = [(a, b) for a, b in zip(HEAD_STYLE.splitlines(), written.splitlines()) if a != b]
    assert changed == [('    - 0.45', '    - 0.56'), ('    - 0.22', '    - 0.30230645')]
    assert yaml.safe_load(written) == yaml.safe_load(YAML_CPP_STAGED)
