"""Datum-to-datum mating on the existing fixed asset-mounting relation."""
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

import asset_mounting as m  # noqa: E402
from physical_destination import rotation, apply  # noqa: E402

BIN_MESH = {'path': 'package://sorting_bin_description/meshes/sorting_bin.stl', 'scale': [0.001] * 3,
            'rpy': [math.pi / 2, 0., math.pi / 2], 'origin_offset': [-0.173899436593, 0., -0.1]}


def cell():
    """Reviewed ur5_2f_test physical records: table surface, standard bin, camera, camera stand."""
    return [
        {'id': 'table', 'pose_xyz': [0.71, -0.2, -0.0009], 'pose_rpy': [0., 0., 0.], 'surface_z_m': 0.0},
        {'id': 'bin', 'catalog_asset_id': 'bin_standard', 'mesh': copy.deepcopy(BIN_MESH),
         'pose_xyz': [0.45, 0.22, 0.1], 'pose_rpy': [0., 0., 0.]},
        {'id': 'stand', 'catalog_asset_id': 'camera_stand', 'mesh': {'path': '/catalog/camera_stand.stl', 'scale': [1., 1., 1.]},
         'pose_xyz': [0.25, -0.45, 0.6], 'pose_rpy': [0., 0., 0.]},
        {'id': 'camera', 'pose_xyz': [0.4, -0.36, 0.85], 'pose_rpy': [0., 1.57, 0.]},
    ]


def by_id(rows):
    return {row['id']: row for row in rows}


def datum_world(rows, asset, datum):
    row = by_id(rows)[asset]
    return m.compose(m.world_pose(row), m.pose(m.anchors(row)[datum]))


def assert_pose(a, b, tol=1e-9):
    assert a[0] == pytest.approx(b[0], abs=tol)
    for row_a, row_b in zip(a[1], b[1]):
        assert row_a == pytest.approx(row_b, abs=tol)


def mate_bin(rows, **extra):
    return m.edit(rows, 'bin', parent_object='table', parent_link='surface', child_datum='bottom',
                  alignment='flush', relative_pose={'xyz': [-0.26, 0.42, 0.], 'rpy': [0., 0., 0.]}, **extra)


def test_flush_plane_mate_seats_bin_bottom_on_table_top():
    rows = mate_bin(cell())
    b = by_id(rows)['bin']
    assert b['mate'] == {'child_datum': 'bottom', 'alignment': 'flush'}
    assert b['parent_object'] == 'table' and b['parent_link'] == 'surface' and b['joint_type'] == 'fixed'
    # Explicit reviewed frame copied into the scene, so the scene never reads the catalog.
    assert b['mounting_anchors']['bottom']['datum_type'] == 'plane'
    assert b['mounting_anchors']['bottom']['source'] == 'catalog:bin_standard'
    assert b['pose_xyz'] == pytest.approx([0.45, 0.22, 0.0991])
    assert b['pose_rpy'] == pytest.approx([0., 0., 0.])
    bottom, top = datum_world(rows, 'bin', 'bottom'), datum_world(rows, 'table', 'surface')
    assert bottom[0] == pytest.approx([0.45, 0.22, -0.0009])
    # Face to face: outward normals oppose, in-plane X axes agree.
    normal_bin, normal_table = [r[2] for r in bottom[1]], [r[2] for r in top[1]]
    assert normal_bin == pytest.approx([-v for v in normal_table])
    assert [r[0] for r in bottom[1]] == pytest.approx([r[0] for r in top[1]])
    m.validate(rows)


def test_offset_is_expressed_in_the_parent_datum_frame():
    rows = mate_bin(cell())
    rows = m.edit(rows, 'bin', relative_pose={'xyz': [-0.26, 0.42, 0.05], 'rpy': [0., 0., math.pi / 2]})
    b = by_id(rows)['bin']
    assert b['pose_xyz'] == pytest.approx([0.45, 0.22, 0.1491])  # 5 cm gap along the table normal
    assert b['pose_rpy'] == pytest.approx([0., 0., math.pi / 2])  # yaw about the table normal


def test_coincident_plane_mate_aligns_normals_and_flips_the_bin():
    rows = m.edit(cell(), 'bin', parent_object='table', parent_link='surface', child_datum='bottom',
                  alignment='coincident', relative_pose={'xyz': [0., 0., 0.], 'rpy': [0., 0., 0.]})
    bottom, top = datum_world(rows, 'bin', 'bottom'), datum_world(rows, 'table', 'surface')
    assert_pose(bottom, top)
    assert by_id(rows)['bin']['pose_xyz'][2] == pytest.approx(-0.1009)  # upside down below the surface


def test_parent_move_and_rotation_propagate_without_drift():
    rows = mate_bin(cell())
    rows = m.edit(rows, 'camera', parent_object='stand', parent_link='top', child_datum='origin',
                  alignment='coincident', relative_pose={'xyz': [0.15, 0., 0.05], 'rpy': [0., math.pi / 2, 0.]})
    baseline = copy.deepcopy(rows)
    relation = {r['id']: {k: r.get(k) for k in m.RELATION_FIELDS} for r in rows}
    for step in range(25):
        rows = m.edit(rows, 'table', world_pose={'xyz': [0.71 + 0.01 * step, -0.2, -0.0009],
                                                  'rpy': [0., 0., 0.35 * (step % 3)]})
        bottom, top = datum_world(rows, 'bin', 'bottom'), datum_world(rows, 'table', 'surface')
        flush = m.compose(top, m.compose(m.origin_pose(by_id(rows)['bin']), m.alignment_pose('flush')))
        assert_pose(bottom, flush)
        assert by_id(rows)['camera']['pose_xyz'] == by_id(baseline)['camera']['pose_xyz']  # independent relation
    rows = m.edit(rows, 'table', world_pose={'xyz': [0.71, -0.2, -0.0009], 'rpy': [0., 0., 0.]})
    for row, original in zip(rows, baseline):
        assert row['pose_xyz'] == pytest.approx(original['pose_xyz'], abs=1e-12)
        assert row['pose_rpy'] == pytest.approx(original['pose_rpy'], abs=1e-12)
    assert {r['id']: {k: r.get(k) for k in m.RELATION_FIELDS} for r in rows} == relation


def test_rotated_parent_carries_child_rotation():
    rows = mate_bin(cell())
    rows = m.edit(rows, 'table', world_pose={'xyz': [1., 0., 0.], 'rpy': [0., 0., math.pi / 2]})
    b = by_id(rows)['bin']
    assert b['pose_rpy'] == pytest.approx([0., 0., math.pi / 2])
    expected = [1. + apply(rotation([0, 0, math.pi / 2]), [-0.26, 0.42, 0.])[0],
                apply(rotation([0, 0, math.pi / 2]), [-0.26, 0.42, 0.])[1], 0.1]
    assert b['pose_xyz'] == pytest.approx(expected)


def test_attach_without_snap_keeps_world_pose_and_detach_removes_mate():
    rows = m.edit(cell(), 'bin', parent_object='table', parent_link='surface', child_datum='bottom', alignment='flush')
    assert by_id(rows)['bin']['pose_xyz'] == pytest.approx([0.45, 0.22, 0.1])
    assert by_id(rows)['bin']['origin']['z'] == pytest.approx(0.0009)  # bottom hovers 0.9 mm above the top
    mated = by_id(rows)['bin']['pose_xyz']
    rows = m.edit(rows, 'bin', parent_object='stand', parent_link='top', child_datum='bottom', alignment='flush')
    assert by_id(rows)['bin']['pose_xyz'] == pytest.approx(mated)  # reparent keeps world pose
    rows = m.edit(rows, 'bin', parent_object='world', parent_link='world')
    b = by_id(rows)['bin']
    assert b['pose_xyz'] == pytest.approx(mated)
    assert 'mate' not in b and 'origin' not in b
    assert b['mounting_anchors']['bottom']['datum_type'] == 'plane'  # the datum stays an asset property


def test_origin_coincident_keeps_the_v1_data_format():
    rows = m.edit(cell(), 'bin', parent_object='table', parent_link='surface', child_datum='origin', alignment='coincident')
    assert 'mate' not in by_id(rows)['bin']
    v1 = m.edit(cell(), 'bin', parent_object='table', parent_link='surface')
    assert rows == v1


@pytest.mark.parametrize('request_, message', [
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'origin', 'alignment': 'flush'}, 'flush needs two plane datums'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'centre_axis', 'alignment': 'coincident'}, 'axis datum cannot coincide with a plane'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'lid', 'alignment': 'flush'}, "missing datum 'lid'"),
    ({'parent_object': 'table', 'parent_link': 'edge', 'child_datum': 'bottom', 'alignment': 'flush'}, 'invalid anchor'),
    ({'parent_object': 'absent', 'parent_link': 'origin', 'child_datum': 'bottom', 'alignment': 'flush'}, 'missing parent'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'bottom', 'alignment': 'glued'}, 'unsupported alignment'),
    ({'child_datum': 'bottom', 'alignment': 'flush'}, 'select a parent asset'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'bottom', 'alignment': 'flush',
      'relative_pose': {'xyz': [float('nan'), 0., 0.], 'rpy': [0., 0., 0.]}}, 'finite'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'bottom', 'alignment': 'flush',
      'relative_pose': {'xyz': [0., 0., 0.], 'rpy': [0., float('inf'), 0.]}}, 'finite'),
    ({'parent_object': 'table', 'parent_link': 'surface', 'child_datum': 'bottom', 'alignment': 'flush',
      'relative_pose': {'xyz': [0., 5000., 0.], 'rpy': [0., 0., 0.]}}, 'within 1000 m'),
])
def test_invalid_mates_are_rejected_atomically(request_, message):
    rows = cell()
    before = copy.deepcopy(rows)
    with pytest.raises(ValueError, match=message):
        m.edit(rows, 'bin', **request_)
    assert rows == before


def test_cycles_and_over_constraint_are_rejected():
    rows = mate_bin(cell())
    with pytest.raises(ValueError, match='cycle'):
        m.edit(rows, 'table', parent_object='bin', parent_link='rim')
    for corrupt, message in (({'mates': [{'child_datum': 'bottom'}]}, 'over-constrain'),
                             ({'parent_object': ['table', 'stand']}, 'over-constrain'),
                             ({'mate': {'child_datum': 'bottom', 'alignment': 'flush', 'second_parent': 'stand'}}, 'one mate per asset'),
                             ({'parent_object': 'world', 'parent_link': 'world'}, 'needs a parent asset')):
        broken = copy.deepcopy(rows)
        by_id(broken)['bin'].update(corrupt)
        with pytest.raises(ValueError, match=message):
            m.validate(broken)


def test_stale_child_projection_is_rejected():
    rows = mate_bin(cell())
    by_id(rows)['table']['pose_xyz'][0] += 0.1
    with pytest.raises(ValueError, match='stale world pose'):
        m.validate(rows)


def test_catalog_datums_require_the_reviewed_mesh_calibration():
    rows = cell()
    assert set(m.catalog_datums(by_id(rows)['bin'])) == {'bottom', 'rim', 'centre_axis'}
    by_id(rows)['bin']['mesh']['origin_offset'] = [0., 0., 0.]
    assert m.catalog_datums(by_id(rows)['bin']) == {}
    with pytest.raises(ValueError, match="missing datum 'bottom'"):
        mate_bin(rows)


def test_unknown_datum_type_is_rejected():
    rows = cell()
    by_id(rows)['table']['mounting_anchors'] = {'edge': {'pose_xyz': [0, 0, 0], 'pose_rpy': [0, 0, 0], 'datum_type': 'curve'}}
    with pytest.raises(ValueError, match='unsupported type'):
        m.validate(rows)


def test_cli_lists_typed_datums_and_returns_mate_metadata():
    rows = [dict(row, mounting_yaml=json.dumps({k: row[k] for k in ('surface_z_m',) if k in row})) for row in cell()]
    for row in rows:
        row.pop('surface_z_m', None)
    run = lambda request: json.loads(subprocess.run([sys.executable, str(ROOT / 'scripts/asset_mounting.py')],
                                                    input=json.dumps(request), capture_output=True, text=True).stdout)
    listed = run({'operation': 'anchors', 'items': copy.deepcopy(rows)})
    assert listed['ok']
    assert listed['result']['table']['surface']['datum_type'] == 'plane'
    assert listed['result']['bin']['bottom']['catalog_asset_id'] == 'bin_standard'
    assert listed['result']['camera']['origin']['datum_type'] == 'origin'
    edited = run({'id': 'bin', 'items': copy.deepcopy(rows), 'parent_object': 'table', 'parent_link': 'surface',
                  'child_datum': 'bottom', 'alignment': 'flush',
                  'relative_pose': {'xyz': [-0.26, 0.42, 0.], 'rpy': [0., 0., 0.]}})
    assert edited['ok']
    metadata = json.loads(by_id(edited['result'])['bin']['mounting_yaml'])
    assert metadata['mate'] == {'child_datum': 'bottom', 'alignment': 'flush'}
    assert metadata['mounting_anchors']['bottom']['pose_xyz'] == [0., 0., -0.1]
    failed = run({'id': 'bin', 'items': copy.deepcopy(rows), 'parent_object': 'table', 'parent_link': 'surface',
                  'child_datum': 'origin', 'alignment': 'flush'})
    assert failed == {'ok': False, 'error': failed['error']} and 'flush needs two plane datums' in failed['error']


def _scene(tmp_path, rows):
    """A minimal authored cell whose destination zone is owned by the bin."""
    scene = tmp_path / 'cell'
    (scene / 'layout').mkdir(parents=True)
    physical = {'support_surfaces': [], 'assets': [], 'task_zones': []}
    for row in rows:
        record = dict(copy.deepcopy(row), frame='world', geometry_type='box', dimensions=[.3, .2, .2],
                      collision={'enabled': True, 'mode': 'box_proxy'}, type='object', category='environment')
        record.pop('mesh', None)
        if record['id'] == 'bin':
            record.update(type='target_bin', mesh=copy.deepcopy(BIN_MESH), geometry_type='mesh',
                          usable_placement={'pose_xyz': [0., 0., 0.01], 'pose_rpy': [0., 0., 0.], 'dimensions': [.24, .12, .14]})
        physical['assets'].append(record)
    zone = {'id': 'drop', 'type': 'place_zone', 'role': 'place_zone', 'category': 'zone', 'frame': 'world',
            'target_ref': 'bin', 'pose_xyz': [0.45, 0.22, 0.11], 'pose_rpy': [0., 0., 0.], 'dimensions': [.2, .1, .12],
            'placement_local': {'pose_xyz': [0., 0., 0.01], 'pose_rpy': [0., 0., 0.], 'dimensions': [.2, .1, .12]}}
    physical['task_zones'].append(zone)
    (scene / 'environment.yaml').write_text(yaml.safe_dump({'environment': physical}))
    return scene, physical


def _save(scene, physical, rows):
    """Write the editor layout for rows (assets) plus the retained zone, then project it."""
    from workcell_studio_layout_merge import merge
    records = by_id(physical['assets'])
    items = []
    for row in rows:
        item = dict(copy.deepcopy(records[row['id']]), **{k: copy.deepcopy(v) for k, v in row.items() if k != 'mesh'})
        for key in m.RELATION_FIELDS:
            if key not in row:
                item.pop(key, None)
        item['pose'] = {'xyz': row['pose_xyz'], 'rpy': row['pose_rpy']}
        items.append(item)
    saved_env = yaml.safe_load((scene / 'environment.yaml').read_text())['environment']
    zone = copy.deepcopy(saved_env['task_zones'][0])
    zone['pose'] = {'xyz': zone['pose_xyz'], 'rpy': zone['pose_rpy']}
    items.append(zone)
    (scene / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': items}))
    merge(scene, save_authored=True)
    return yaml.safe_load((scene / 'environment.yaml').read_text())['environment']


def test_save_reopen_and_destination_zone_keep_target_local_offset(tmp_path):
    from physical_destination import resolve_destination
    from generate_workcell_from_cell_definition import _build_environment_objects
    from generate_moveit_collision_manifest import load_and_build
    rows = mate_bin(cell())[:3]
    scene, physical = _scene(tmp_path, cell()[:3])
    saved = _save(scene, physical, rows)
    m.validate_physical(saved)
    stored = by_id(saved['assets'])['bin']
    assert stored['mate'] == {'child_datum': 'bottom', 'alignment': 'flush'}
    assert stored['mounting_anchors']['bottom']['pose_rpy'] == pytest.approx([math.pi, 0., 0.])
    local = copy.deepcopy(saved['task_zones'][0]['placement_local'])
    # Rotate/move the parent table and save: the bin and its zone follow.
    rows = m.edit(rows, 'table', world_pose={'xyz': [0.8, -0.1, 0.], 'rpy': [0., 0., 0.4]})
    saved = _save(scene, physical, rows)
    assert saved['task_zones'][0]['placement_local'] == local
    resolve_destination(saved, 'drop')  # projection is current, not stale
    # Three consecutive bin moves, each saved (the PR #3201 regression scenario).
    for y in (-0.02, -0.07, -0.01):
        bin_world = m.world_pose(by_id(rows)['bin'])
        rows = m.edit(rows, 'bin', world_pose={'xyz': [bin_world[0][0], y, bin_world[0][2]], 'rpy': m.angles(bin_world[1])})
        saved = _save(scene, physical, rows)
        assert by_id(saved['assets'])['bin']['pose_xyz'][1] == pytest.approx(y)
        assert by_id(saved['assets'])['bin']['mate'] == {'child_datum': 'bottom', 'alignment': 'flush'}
        assert saved['task_zones'][0]['placement_local'] == local
        resolve_destination(saved, 'drop')
    # Reopen: the relation alone reproduces every saved world pose.
    reopened = copy.deepcopy(saved)
    resolved = m.validate_physical(reopened)
    for record in reopened['assets']:
        assert resolved[record['id']][0] == pytest.approx(record['pose_xyz'], abs=1e-9)
    objects = _build_environment_objects({'environment': saved})['objects']
    generated = next(obj for obj in objects if obj['id'] == 'bin')
    assert generated['mate'] == stored['mate'] and generated['parent_object'] == 'table'
    manifest = load_and_build(scene / 'layout/workcell_studio_layout.yaml')
    collision = next(obj for obj in manifest['objects'] if obj['source_item_id'] == 'bin')
    assert collision['pose']['xyz'] is not None


def test_failed_mate_save_leaves_environment_unchanged(tmp_path):
    from workcell_studio_layout_merge import merge
    rows = mate_bin(cell())[:3]
    scene, physical = _scene(tmp_path, cell()[:3])
    _save(scene, physical, rows)
    before = (scene / 'environment.yaml').read_bytes()
    layout = yaml.safe_load((scene / 'layout/workcell_studio_layout.yaml').read_text())
    for item in layout['items']:
        if item['id'] == 'bin':
            item['mate'] = {'child_datum': 'origin', 'alignment': 'flush'}
    (scene / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump(layout))
    with pytest.raises(ValueError, match='flush needs two plane datums'):
        merge(scene, save_authored=True)
    assert (scene / 'environment.yaml').read_bytes() == before


def test_cli_reads_reviewed_calibration_from_the_saved_layout(tmp_path):
    """The editor does not track mesh RPY/offset; Save keeps the layout's mesh mapping."""
    scene = tmp_path / 'cell'
    (scene / 'layout').mkdir(parents=True)
    (scene / 'layout/workcell_studio_layout.yaml').write_text(yaml.safe_dump({'items': [
        {'id': 'bin', 'catalog_asset_id': 'bin_standard', 'mesh': copy.deepcopy(BIN_MESH)}]}))
    editor_row = {'id': 'bin', 'catalog_asset_id': 'bin_standard', 'mesh': {'scale': [0.001] * 3, 'rpy': [0, 0, 0]},
                  'pose_xyz': [0.45, 0.22, 0.1], 'pose_rpy': [0, 0, 0], 'mounting_yaml': '{}'}
    run = lambda request: json.loads(subprocess.run([sys.executable, str(ROOT / 'scripts/asset_mounting.py')],
                                                    input=json.dumps(request), capture_output=True, text=True).stdout)
    assert 'bottom' not in run({'operation': 'anchors', 'items': [dict(editor_row)]})['result']['bin']
    listed = run({'operation': 'anchors', 'items': [dict(editor_row)], 'scene_dir': str(scene)})
    assert listed['result']['bin']['bottom']['datum_type'] == 'plane'
