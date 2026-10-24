"""Environment asset STLs are wound outward (counter-clockwise seen from outside).

Web3D draws and ray-picks front faces only. An inside-out mesh shows its far
inner walls, and a Mate click on its top reaches an inner wall instead of the
top face, so the click is refused. The reviewed placeholder boxes (for example
the Asset Library camera_stand) were once stored inside out."""
import math
import re
import struct
from collections import Counter
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT_ROOTS = [ROOT / 'workcell_builder' / 'workcell_builder' / 'assets' / 'environment', ROOT / 'assets' / 'environment']
CAMERA_STAND = ROOT / 'workcell_builder/workcell_builder/assets/environment/camera_stand_description/meshes/camera_stand.stl'
NUMBER = r'([-+0-9.eE]+)'
FACET = re.compile(r'facet\s+normal\s+' + r'\s+'.join([NUMBER] * 3) + r'\s+outer\s+loop' + (r'\s+vertex\s+' + r'\s+'.join([NUMBER] * 3)) * 3)


def read_stl(path):
    """[(stored_normal, (a, b, c))] from an ASCII or binary STL."""
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and 84 + 50 * count == len(data):
        facets = []
        for i in range(count):
            v = struct.unpack_from('<12f', data, 84 + 50 * i)
            facets.append((v[0:3], (v[3:6], v[6:9], v[9:12])))
        return facets
    facets = []
    for m in FACET.finditer(data.decode('ascii', errors='replace')):
        v = [float(x) for x in m.groups()]
        facets.append((tuple(v[0:3]), (tuple(v[3:6]), tuple(v[6:9]), tuple(v[9:12]))))
    return facets


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def winding_normal(tri):
    a, b, c = tri
    return cross(sub(b, a), sub(c, a))


def closed_and_consistent(facets):
    """Every directed edge is matched by exactly one reversed edge."""
    edges = Counter()
    for _normal, tri in facets:
        for i in range(3):
            edges[(tri[i], tri[(i + 1) % 3])] += 1
    return bool(edges) and all(n == 1 and edges.get((b, a)) == 1 for (a, b), n in edges.items())


def signed_volume(facets):
    return sum(dot(a, cross(b, c)) for _normal, (a, b, c) in facets) / 6.0


def environment_meshes():
    # Large vendor scans (100k+ triangles) are not placeholder assets; skip them to keep this fast.
    return sorted(p for root in ENVIRONMENT_ROOTS for p in root.rglob('*')
                  if p.suffix.lower() == '.stl' and p.stat().st_size < 2_000_000)


def test_closed_environment_meshes_are_wound_outward():
    checked, inside_out = [], []
    for path in environment_meshes():
        facets = read_stl(path)
        if not facets or not closed_and_consistent(facets):
            continue  # open or non-manifold meshes have no inside
        checked.append(path)
        if signed_volume(facets) <= 0.0:
            inside_out.append(str(path.relative_to(ROOT)))
    assert CAMERA_STAND in checked
    assert len(checked) >= 24
    assert not inside_out, f'inside-out (inward-wound) meshes: {inside_out}'


def test_stored_facet_normals_agree_with_winding():
    for path in environment_meshes():
        for normal, tri in read_stl(path):
            if not any(normal):
                continue  # readers recompute zero normals from the winding
            assert dot(normal, winding_normal(tri)) > 0.0, f'{path.relative_to(ROOT)}: facet normal opposes its winding'


def test_camera_stand_faces_match_its_reviewed_datums():
    """The winding fix keeps the reviewed geometry: the outward faces lie on the
    reviewed top/base datum planes and point along their normals."""
    facets = read_stl(CAMERA_STAND)
    assert len(facets) == 12
    points = [p for _n, tri in facets for p in tri]
    for axis, (lo, hi) in enumerate([(-0.04, 0.04), (-0.04, 0.04), (-0.6, 0.6)]):
        assert min(p[axis] for p in points) == pytest.approx(lo)
        assert max(p[axis] for p in points) == pytest.approx(hi)
    datums = yaml.safe_load((ROOT / 'catalog/workcell_studio_asset_datums.yaml').read_text())['assets']['camera_stand']
    assert datums['calibration']['scale'] == [1.0, 1.0, 1.0]
    for name in ('top', 'base'):
        datum = datums['datums'][name]
        roll, pitch, yaw = datum['pose_rpy']
        # A plane datum's outward normal is its local +Z (ROS fixed-axis RPY).
        x, y = math.cos(roll) * math.sin(pitch), -math.sin(roll)
        normal = [math.cos(yaw) * x - math.sin(yaw) * y, math.sin(yaw) * x + math.cos(yaw) * y, math.cos(roll) * math.cos(pitch)]
        on_plane = [tri for _n, tri in facets if all(abs(dot(sub(p, datum['pose_xyz']), normal)) < 1e-9 for p in tri)]
        assert len(on_plane) == 2, name
        for tri in on_plane:
            n = winding_normal(tri)
            assert dot(n, normal) / math.sqrt(dot(n, n)) == pytest.approx(1.0), name
