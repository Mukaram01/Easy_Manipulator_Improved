"""Two-part Align & Arrange (scripts/asset_alignment.py) on disposable scenes."""
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

import asset_alignment as alignment  # noqa: E402
import asset_mounting as mounting  # noqa: E402

BIN_MESH = {  # the calibrated sorting bin of ur5_2f_test
    'path': 'package://sorting_bin_description/meshes/sorting_bin.stl',
    'scale': [0.001, 0.001, 0.001],
    'rpy': [math.pi / 2, 0., math.pi / 2],
    'origin_offset': [-0.173899436593, 0., -0.1],
}


def scene(tmp_path, items):
    (tmp_path / 'layout').mkdir(parents=True)
    (tmp_path / 'layout' / 'workcell_studio_layout.yaml').write_text(
        yaml.safe_dump({'schema_version': 'workcell_studio_layout/v1', 'items': items}), encoding='utf-8')
    return str(tmp_path)


def table(yaw=0.):
    return {'id': 'table', 'geometry_type': 'box', 'dimensions': [1.2, 0.8, 0.04], 'surface_z_m': 0.02,
            'pose_xyz': [1., 0., 0.7], 'pose_rpy': [0., 0., yaw]}


def block(xyz=(0.3, 0.3, 1.), size=(0.2, 0.1, 0.1)):
    return {'id': 'block', 'geometry_type': 'box', 'dimensions': list(size), 'pose_xyz': list(xyz), 'pose_rpy': [0., 0., 0.]}


def rows_of(items):
    keep = ('id', 'pose_xyz', 'pose_rpy', 'surface_z_m', 'role', 'type', 'category',
            'parent_object', 'parent_link', 'origin', 'mounting_anchors', 'mate')
    return [{key: copy.deepcopy(item[key]) for key in keep if key in item} for item in items]


def run(items, tmp_path, **request):
    scene_dir = scene(tmp_path, items)
    request.setdefault('id', 'block')
    request.setdefault('reference', 'table')
    return alignment.plan(rows_of(items), request, scene_dir)


def pose_of(result, identifier):
    return next(row for row in result['rows'] if row['id'] == identifier)['pose_xyz']


def test_centre_on_reference_moves_only_the_centred_axes(tmp_path):
    result = run([table(), block()], tmp_path, rules={'x': 'centre', 'y': 'centre'})
    assert pose_of(result, 'block') == pytest.approx([1., 0., 1.])
    report = result['report']
    assert report['delta_reference_m'] == pytest.approx([0.7, -0.3, 0.])
    assert report['distance_m'] == pytest.approx(math.hypot(0.7, 0.3))
    assert report['method'] == 'bounding_box'
    assert result['request'] == {'id': 'block', 'world_pose': {'xyz': pytest.approx([1., 0., 1.]),
                                                               'rpy': pytest.approx([0., 0., 0.])}}


def test_edges_offsets_and_place_on_top(tmp_path):
    items = [table(), block()]
    left = run(items, tmp_path / 'a', rules={'x': 'min', 'y': 'max'}, offset_m=[0.05, -0.01, 0.])
    # Table spans X 0.4..1.6, Y -0.4..0.4; the block is 0.2 x 0.1.
    assert pose_of(left, 'block')[:2] == pytest.approx([0.4 + 0.1 + 0.05, 0.4 - 0.05 - 0.01])
    right = run(items, tmp_path / 'b', rules={'x': 'max', 'y': 'min'})
    assert pose_of(right, 'block')[:2] == pytest.approx([1.6 - 0.1, -0.4 + 0.05])
    top = run(items, tmp_path / 'c', rules={'z': 'on_top'}, offset_m=[0., 0., 0.003])
    # The reviewed surface (surface_z_m 0.02 above the table origin) wins over the box top.
    assert pose_of(top, 'block')[2] == pytest.approx(0.7 + 0.02 + 0.05 + 0.003)
    assert "verified surface datum 'surface'" in top['report']['sources']['top']
    # The offset applies only to axes with a rule.
    assert pose_of(top, 'block')[:2] == pytest.approx([0.3, 0.3])


def test_place_on_top_without_verified_surface_is_labelled_bounding_box(tmp_path):
    reference = dict(table())
    reference.pop('surface_z_m')
    result = run([reference, block()], tmp_path, rules={'z': 'on_top'})
    assert pose_of(result, 'block')[2] == pytest.approx(0.7 + 0.02 + 0.05)
    assert result['report']['sources']['top'] == 'bounding-box top of the reference'


def test_rotated_reference_uses_its_own_axes(tmp_path):
    yaw = math.radians(30.)
    items = [table(yaw), block(xyz=(1.2, 0.2, 1.), size=(0.1, 0.1, 0.1))]
    result = run(items, tmp_path, rules={'x': 'min', 'y': 'centre'})
    xyz = pose_of(result, 'block')
    # Expressed in the table frame, the block sits on its left edge line, centred across it.
    c, s = math.cos(yaw), math.sin(yaw)
    local = [c * (xyz[0] - 1.) + s * xyz[1], -s * (xyz[0] - 1.) + c * xyz[1]]
    # The block is rotated -30 deg relative to the table: its box in the table frame is wider.
    half = 0.05 * (c + s)
    assert local == pytest.approx([-0.6 + half, 0.], abs=1e-9)  # targets are rounded to 1 nm
    assert result['report']['notes'] and 'rotated relative' in result['report']['notes'][0]
    assert next(row for row in result['rows'] if row['id'] == 'block')['pose_rpy'] == pytest.approx([0., 0., 0.])


def test_quarter_turn_reference_is_exact(tmp_path):
    items = [table(math.pi / 2), block(xyz=(1.2, 0.2, 1.))]
    result = run(items, tmp_path, rules={'x': 'max'})
    # Table +X points along world +Y: its right edge is world Y 0.6; the block's 0.2 m side
    # lies along world X, so its extent along table X (world Y) is 0.1.
    assert pose_of(result, 'block') == pytest.approx([1.2, 0.6 - 0.05, 1.])
    assert result['report']['notes'] == []


def test_real_bin_mesh_bounds_follow_the_authored_calibration(tmp_path):
    from generate_moveit_collision_manifest import _resolve_local_mesh, _stl_bounds
    bin_item = {'id': 'bin', 'geometry_type': 'mesh', 'mesh': BIN_MESH, 'pose_xyz': [0.45, 0.22, 0.1],
                'pose_rpy': [0., 0., 0.]}
    scene_dir = scene(tmp_path, [table(), bin_item])
    lo, hi, source = alignment.part_box(bin_item, scene_dir, 'Destination Bin')
    raw_lo, raw_hi = _stl_bounds(_resolve_local_mesh(BIN_MESH['path'], None))
    # Independent check: the 8 raw corners through scale, RPY and offset (exact for quarter turns).
    rot = mounting.rotation(BIN_MESH['rpy'])
    corners = [mounting.apply(rot, [c * 0.001 for c in corner]) for corner in alignment._corners(raw_lo, raw_hi)]
    expected_lo = [min(c[k] for c in corners) + BIN_MESH['origin_offset'][k] for k in range(3)]
    expected_hi = [max(c[k] for c in corners) + BIN_MESH['origin_offset'][k] for k in range(3)]
    assert lo == pytest.approx(expected_lo, abs=1e-9) and hi == pytest.approx(expected_hi, abs=1e-9)
    assert 'sorting_bin.stl' in source
    result = alignment.plan(rows_of([table(), bin_item]),
                            {'id': 'bin', 'reference': 'table', 'rules': {'x': 'centre', 'y': 'centre', 'z': 'on_top'}},
                            scene_dir)
    xyz = pose_of(result, 'bin')
    # The bin's geometry centre, not its origin, lands on the table centre; its bottom on the surface.
    assert xyz[0] + (lo[0] + hi[0]) / 2 == pytest.approx(1.)
    assert xyz[1] + (lo[1] + hi[1]) / 2 == pytest.approx(0.)
    assert xyz[2] + lo[2] == pytest.approx(0.72)


def test_keep_relative_mounts_on_the_reference_and_follows_it(tmp_path):
    items = [table(), block()]
    result = run(items, tmp_path, rules={'x': 'centre'}, keep_relative=True)
    moved = next(row for row in result['rows'] if row['id'] == 'block')
    assert moved['parent_object'] == 'table' and moved['parent_link'] == 'origin' and 'mate' not in moved
    assert result['request']['parent_object'] == 'table'
    follow = mounting.edit(result['rows'], 'table', world_pose={'xyz': [2., 0., 0.7], 'rpy': [0., 0., 0.]})
    assert next(row for row in follow if row['id'] == 'block')['pose_xyz'] == pytest.approx([2., 0.3, 1.])
    # Align once never creates a relation.
    once = run(items, tmp_path / 'once', rules={'x': 'centre'})
    assert 'parent_object' not in next(row for row in once['rows'] if row['id'] == 'block')


def test_mounted_part_keeps_its_parent_and_children_follow(tmp_path):
    items = [table(), block(), {'id': 'sensor', 'geometry_type': 'box', 'dimensions': [0.02, 0.02, 0.02],
                                'pose_xyz': [0.3, 0.3, 1.2], 'pose_rpy': [0., 0., 0.]}]
    rows = mounting.edit(rows_of(items), 'sensor', parent_object='block', parent_link='origin')
    scene_dir = scene(tmp_path, items)
    result = alignment.plan(rows, {'id': 'block', 'reference': 'table', 'rules': {'x': 'centre'}}, scene_dir)
    assert pose_of(result, 'sensor') == pytest.approx([1., 0.3, 1.2])
    # The sensor follows the block, so it cannot be the block's reference.
    with pytest.raises(alignment.AlignmentError, match='mounted on block and moves with it'):
        alignment.plan(rows, {'id': 'block', 'reference': 'sensor', 'rules': {'x': 'centre'}}, scene_dir)
    mounted = alignment.plan(rows, {'id': 'sensor', 'reference': 'table', 'rules': {'y': 'centre'}}, scene_dir)
    sensor = next(row for row in mounted['rows'] if row['id'] == 'sensor')
    assert sensor['parent_object'] == 'block' and sensor['pose_xyz'] == pytest.approx([0.3, 0., 1.2])
    assert 'stays mounted on block' in mounted['report']['notes'][0]


@pytest.mark.parametrize('change, request_extra, message', [
    (None, {'reference': 'block'}, 'cannot be aligned to itself'),
    (None, {'rules': {}}, 'choose at least one'),
    (None, {'rules': {'x': 'on_top'}}, 'unsupported rule'),
    (None, {'offset_m': [20., 0., 0.]}, 'offsets must be within'),
    (lambda items: items[1].update(dimensions=[0.2, 0., 0.1]), {}, 'must be positive'),
    (lambda items: items[1].update(dimensions=[0.2, float('nan'), 0.1]), {}, 'three finite numbers'),
    (lambda items: items[0].update(geometry_type='mesh', mesh={'path': 'package://sorting_bin_description/meshes/none.stl'}),
     {}, 'cannot be found'),
    (lambda items: items[0].update(geometry_type='mesh', mesh=dict(BIN_MESH, scale=[1., 1., 1.], rpy=[0., 0., 0.])),
     {}, 'check its mesh units'),
    (lambda items: items[0].update(geometry_type='cylinder'), {}, 'reads STL meshes and boxes only'),
])
def test_invalid_requests_and_geometry_fail_with_reasons(tmp_path, change, request_extra, message):
    items = [table(), block()]
    if change:
        change(items)
    request = {'rules': {'x': 'centre'}, **request_extra}
    with pytest.raises(alignment.AlignmentError, match=message):
        run(items, tmp_path, **request)


def test_locked_unsaved_and_zone_parts_are_rejected(tmp_path):
    items = [table(), block()]
    scene_dir = scene(tmp_path, items)
    locked = [row for row in rows_of(items) if row['id'] != 'block']  # locked parts are not editable rows
    with pytest.raises(alignment.AlignmentError, match='locked or not an editable'):
        alignment.plan(locked, {'id': 'block', 'reference': 'table', 'rules': {'x': 'centre'}}, scene_dir)
    unsaved = rows_of(items) + [{'id': 'new_part', 'pose_xyz': [0., 0., 0.], 'pose_rpy': [0., 0., 0.]}]
    with pytest.raises(alignment.AlignmentError, match='Save Layout, then align'):
        alignment.plan(unsaved, {'id': 'new_part', 'reference': 'table', 'rules': {'x': 'centre'}}, scene_dir)
    zone = rows_of(items) + [{'id': 'place_zone', 'type': 'place_zone', 'pose_xyz': [0., 0., 0.], 'pose_rpy': [0., 0., 0.]}]
    with pytest.raises(alignment.AlignmentError, match='task zone'):
        alignment.plan(zone, {'id': 'block', 'reference': 'place_zone', 'rules': {'x': 'centre'}}, scene_dir)
    with pytest.raises(alignment.AlignmentError, match='unsaved dimension edits'):
        alignment.plan(rows_of(items), {'id': 'block', 'reference': 'table', 'rules': {'x': 'centre'},
                                        'live_dimensions': {'block': [0.3, 0.1, 0.1]}}, scene_dir)
    with pytest.raises(alignment.AlignmentError, match='cannot follow it'):
        alignment.plan(rows_of(items)[1:], {'id': 'block', 'reference': 'table', 'rules': {'x': 'centre'},
                                            'keep_relative': True,
                                            'reference_world': {'xyz': [1., 0., 0.7], 'rpy': [0., 0., 0.]}}, scene_dir)


def test_locked_reference_is_read_from_its_reported_world_pose(tmp_path):
    items = [table(), block()]
    scene_dir = scene(tmp_path, items)
    result = alignment.plan(rows_of(items)[1:], {'id': 'block', 'reference': 'table', 'rules': {'y': 'centre'},
                                                 'reference_world': {'xyz': [1., 0., 0.7], 'rpy': [0., 0., 0.]}}, scene_dir)
    assert pose_of(result, 'block') == pytest.approx([0.3, 0., 1.])


def test_resolver_cli_plans_alignment_without_writing_files(tmp_path):
    items = [table(), block()]
    scene_dir = scene(tmp_path, items)
    layout = Path(scene_dir) / 'layout' / 'workcell_studio_layout.yaml'
    before = layout.read_bytes()
    rows = [dict(row, mounting_yaml='') for row in rows_of(items)]
    request = {'operation': 'align', 'id': 'block', 'reference': 'table', 'rules': {'x': 'centre', 'z': 'on_top'},
               'items': rows, 'scene_dir': scene_dir, 'names': {'block': 'Block', 'table': 'Workbench'}}
    done = subprocess.run([sys.executable, str(ROOT / 'scripts/asset_mounting.py')], input=json.dumps(request),
                          capture_output=True, text=True, check=True)
    response = json.loads(done.stdout)
    assert response['ok'] and response['result']['request']['world_pose']['xyz'] == pytest.approx([1., 0.3, 0.77])
    assert all('mounting_yaml' in row for row in response['result']['rows'])
    # The planned request is exactly what Apply sends through the ordinary edit path.
    applied = subprocess.run([sys.executable, str(ROOT / 'scripts/asset_mounting.py')],
                             input=json.dumps({'items': rows, **response['result']['request']}),
                             capture_output=True, text=True, check=True)
    assert json.loads(applied.stdout)['result'] == response['result']['rows']
    request['reference'] = 'block'
    failed = subprocess.run([sys.executable, str(ROOT / 'scripts/asset_mounting.py')], input=json.dumps(request),
                            capture_output=True, text=True)
    assert failed.returncode == 2 and 'Block cannot be aligned to itself' in json.loads(failed.stdout)['error']
    assert layout.read_bytes() == before
