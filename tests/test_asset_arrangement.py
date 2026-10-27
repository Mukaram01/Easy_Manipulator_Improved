"""Multi-object arrangement (scripts/asset_arrangement.py) on disposable scenes."""
import copy
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import asset_arrangement as arrangement  # noqa: E402
import asset_mounting as mounting  # noqa: E402


def box(identifier, xyz, size=(0.1, 0.1, 0.1), yaw=0.):
    return {'id': identifier, 'geometry_type': 'box', 'dimensions': list(size), 'pose_xyz': list(xyz),
            'pose_rpy': [0., 0., yaw]}


def cell(tmp_path, items):
    (tmp_path / 'layout').mkdir(parents=True, exist_ok=True)
    (tmp_path / 'layout' / 'workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': items}))
    keep = ('id', 'pose_xyz', 'pose_rpy', 'surface_z_m', 'type', 'role', 'parent_object', 'parent_link', 'origin')
    return [{key: copy.deepcopy(item[key]) for key in keep if key in item} for item in items], str(tmp_path)


def pose(result, identifier):
    return next(row for row in result['rows'] if row['id'] == identifier)['pose_xyz']


def run(tmp_path, items, op, members, reference='table', **params):
    rows, scene = cell(tmp_path, items)
    return arrangement.plan(rows, {'op': op, 'reference': reference, 'members': members, 'params': params}, scene)


def three_bins():
    return [box('table', [1., 0., 0.7], (1.2, 0.8, 0.04)), box('a', [0.6, 0.3, 1.]), box('b', [0.9, -0.2, 1.]),
            box('c', [1.3, 0.1, 1.], (0.2, 0.1, 0.1))]


def test_align_moves_every_member_in_one_transaction(tmp_path):
    result = run(tmp_path, three_bins(), 'align', ['a', 'b', 'c'], rules={'y': 'max'})
    for identifier, half in (('a', 0.05), ('b', 0.05), ('c', 0.05)):
        assert pose(result, identifier)[1] == pytest.approx(0.4 - half)
    assert pose(result, 'table') == [1., 0., 0.7]
    assert {entry['id'] for entry in result['report']['moved']} == {'a', 'b', 'c'}


def test_equal_spacing_keeps_the_reference_and_orders_by_position(tmp_path):
    items = [box('ref', [0., 0., 0.]), box('a', [0.25, 0., 0.]), box('b', [1., 0., 0.], (0.3, 0.1, 0.1)),
             box('c', [0.6, 0., 0.])]
    result = run(tmp_path, items, 'distribute', ['a', 'b', 'c'], reference='ref', axis='x', mode='equal')
    report = result['report']
    assert report['order'] == ['ref', 'a', 'c', 'b']
    # Span -0.05..1.15 = 1.2; sizes 0.1+0.1+0.1+0.3 = 0.6; three equal gaps of 0.2.
    assert report['gap_m'] == pytest.approx(0.2)
    assert pose(result, 'ref') == [0., 0., 0.]
    assert pose(result, 'a')[0] == pytest.approx(0.3)
    assert pose(result, 'c')[0] == pytest.approx(0.6)
    assert pose(result, 'b')[0] == pytest.approx(1.0)


def test_exact_gap_from_an_interior_reference(tmp_path):
    items = [box('ref', [0.5, 0., 0.]), box('a', [0., 0., 0.]), box('b', [0.9, 0., 0.])]
    result = run(tmp_path, items, 'distribute', ['a', 'b'], reference='ref', axis='x', mode='gap', gap_m=0.02)
    assert pose(result, 'ref') == [0.5, 0., 0.]
    assert pose(result, 'a')[0] == pytest.approx(0.5 - 0.1 - 0.02)
    assert pose(result, 'b')[0] == pytest.approx(0.5 + 0.1 + 0.02)


def test_distribution_follows_a_rotated_reference(tmp_path):
    yaw = math.radians(30.)
    c, s = math.cos(yaw), math.sin(yaw)
    along = lambda d: [d * c, d * s, 0.]  # noqa: E731
    items = [box('ref', [0., 0., 0.], yaw=yaw), box('a', along(0.4), yaw=yaw), box('b', along(0.15), yaw=yaw)]
    result = run(tmp_path, items, 'distribute', ['a', 'b'], reference='ref', axis='x', mode='gap', gap_m=0.05)
    assert pose(result, 'b') == pytest.approx(along(0.15), abs=1e-9)
    assert pose(result, 'a') == pytest.approx(along(0.30), abs=1e-9)


def test_move_in_reference_and_local_axes_and_rotation_about_the_reference(tmp_path):
    items = [box('ref', [1., 0., 0.], yaw=math.pi / 2), box('a', [1., 1., 0.]), box('b', [2., 0., 0.], yaw=math.pi / 2)]
    moved = run(tmp_path, items, 'move', ['a', 'b'], reference='ref', frame='reference', translate_m=[0.1, 0., 0.])
    assert pose(moved, 'a') == pytest.approx([1., 1.1, 0.])  # reference +X is world +Y
    local = run(tmp_path / 'local', items, 'move', ['a', 'b'], reference='ref', frame='local', translate_m=[0.1, 0., 0.])
    assert pose(local, 'a') == pytest.approx([1.1, 1., 0.]) and pose(local, 'b') == pytest.approx([2., 0.1, 0.])
    turned = run(tmp_path / 'turn', items, 'move', ['a'], reference='ref', frame='world', pivot='reference',
                 rotate_deg=[0., 0., 90.])
    a = next(row for row in turned['rows'] if row['id'] == 'a')
    assert a['pose_xyz'] == pytest.approx([0., 0., 0.], abs=1e-9)
    assert a['pose_rpy'] == pytest.approx([0., 0., math.pi / 2])


def test_snap_grid_rounds_origins_horizontally(tmp_path):
    items = [box('ref', [0., 0., 0.]), box('a', [0.123456, -0.0517, 0.333])]
    result = run(tmp_path, items, 'snap_grid', ['a'], reference='ref', step_m=0.01)
    assert pose(result, 'a') == [0.12, -0.05, 0.333]


def test_mount_and_detach_are_atomic_and_keep_world_poses(tmp_path):
    items = three_bins()
    mounted = run(tmp_path, items, 'mount', ['a', 'b'])
    for identifier in ('a', 'b'):
        row = next(row for row in mounted['rows'] if row['id'] == identifier)
        assert row['parent_object'] == 'table' and row['pose_xyz'] == pytest.approx(
            next(item for item in items if item['id'] == identifier)['pose_xyz'])
    follow = mounting.edit(mounted['rows'], 'table', world_pose={'xyz': [1.1, 0., 0.7], 'rpy': [0., 0., 0.]})
    assert next(row for row in follow if row['id'] == 'a')['pose_xyz'] == pytest.approx([0.7, 0.3, 1.])
    rows, scene = cell(tmp_path / 'detach', items)
    detached = arrangement.plan(mounted['rows'], {'op': 'detach', 'reference': 'table', 'members': ['a', 'b']}, scene)
    assert all('parent_object' not in row or row['parent_object'] == 'world' for row in detached['rows'])
    assert detached['report']['moved'] == []


@pytest.mark.parametrize('op, members, params, message', [
    ('align', [], {'rules': {'x': 'centre'}}, 'select at least one part besides the reference'),
    ('align', ['ghost'], {'rules': {'x': 'centre'}}, 'locked or not an editable'),
    ('distribute', ['a'], {'axis': 'x', 'mode': 'equal'}, 'at least three parts'),
    ('distribute', ['a', 'b'], {'axis': 'w'}, 'choose the distribution axis'),
    ('move', ['a'], {'translate_m': [0., 0., 0.]}, 'enter a distance or an angle'),
    ('move', ['a'], {'translate_m': [20., 0., 0.]}, 'within 10 m'),
    ('snap_grid', ['a'], {'step_m': 0.}, 'between 0.1 mm and 1000 mm'),
    ('detach', ['a'], {}, 'nothing to detach'),
    ('explode', ['a'], {}, 'unsupported arrangement'),
])
def test_refusals_change_nothing(tmp_path, op, members, params, message):
    with pytest.raises(arrangement.ArrangementError, match=message):
        run(tmp_path, three_bins(), op, members, **params)


def test_hierarchy_conflicts_are_refused(tmp_path):
    rows, scene = cell(tmp_path, three_bins())
    rows = mounting.edit(rows, 'b', parent_object='a', parent_link='origin')
    request = {'op': 'move', 'reference': 'table', 'members': ['a', 'b'], 'params': {'translate_m': [0.1, 0., 0.]}}
    with pytest.raises(arrangement.ArrangementError, match='b is mounted on a'):
        arrangement.plan(rows, request, scene)
    request = {'op': 'move', 'reference': 'b', 'members': ['a'], 'params': {'translate_m': [0.1, 0., 0.]}}
    with pytest.raises(arrangement.ArrangementError, match='b is mounted on a and would move with it'):
        arrangement.plan(rows, request, scene)


def test_cli_plan_equals_commit_and_never_writes(tmp_path):
    rows, scene = cell(tmp_path, three_bins())
    layout = Path(scene) / 'layout' / 'workcell_studio_layout.yaml'
    before = layout.read_bytes()
    request = {'operation': 'arrange', 'op': 'distribute', 'reference': 'table', 'members': ['a', 'b', 'c'],
               'params': {'axis': 'x', 'mode': 'gap', 'gap_m': 0.01}, 'items': [dict(row, mounting_yaml='') for row in rows],
               'scene_dir': scene}
    script = str(ROOT / 'scripts/asset_mounting.py')
    first = json.loads(subprocess.run([sys.executable, script], input=json.dumps(request), capture_output=True,
                                      text=True, check=True).stdout)
    second = json.loads(subprocess.run([sys.executable, script], input=json.dumps(request), capture_output=True,
                                       text=True, check=True).stdout)
    assert first == second and first['ok']  # deterministic: preview == apply
    assert all('mounting_yaml' in row for row in first['result']['rows'])
    assert layout.read_bytes() == before
