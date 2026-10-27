#!/usr/bin/env python3
"""Fixed asset joints in the existing physical scene contract.

parent_object/parent_link/origin own a mounted pose; pose_xyz/pose_rpy are
checked world projections for existing scene, collision and destination readers.
No robot/tool joints or mesh calibration transforms are changed here.

Datum mating (v2) is the same relation with two optional parts: a typed datum
(origin, point, axis or plane) for each named anchor, and a `mate` mapping on
the child naming its own datum and the alignment. The authored `origin` is then
the offset of the child datum from the parent datum, expressed in the parent
datum frame:

    child_world = parent_world * parent_datum * origin * alignment * child_datum^-1

`coincident` aligns the datum frames; `flush` (plane to plane) turns the child
datum about its X axis so the outward normals oppose (face to face). Without a
`mate` mapping the v1 formula applies unchanged.
"""
from __future__ import annotations
import copy
import json
import math
import sys
from pathlib import Path

from physical_destination import angles, apply, multiply, pose, rotation, transpose, vector


def world_pose(row):
    value = row.get("pose", {})
    return pose({"pose_xyz": row.get("pose_xyz", value.get("xyz", [value.get(k, 0.) for k in ("x", "y", "z")])),
                 "pose_rpy": row.get("pose_rpy", value.get("rpy", [value.get(k, 0.) for k in ("roll", "pitch", "yaw")]))})

RELATION_FIELDS = ('parent_object', 'parent_link', 'joint_type', 'origin', 'mounting_anchors', 'mate')
COLLECTIONS = ('support_surfaces', 'assets', 'sensors', 'task_zones')
DATUM_TYPES = ('origin', 'point', 'axis', 'plane')
ALIGNMENTS = ('coincident', 'flush')
OFFSET_LIMIT_M = 1000.
DATUM_CATALOG = Path(__file__).resolve().parents[1] / 'catalog' / 'workcell_studio_asset_datums.yaml'


def semantic_zone(row):
    return row.get('category') == 'zone' or row.get('type') in ('zone', 'pick_zone', 'place_zone', 'safety_zone', 'drop_zone') or row.get('role') in ('pick_zone', 'place_zone', 'safety_zone')


def compose(a, b):
    ap, ar = a
    bp, br = b
    return [x+y for x, y in zip(ap, apply(ar, bp))], multiply(ar, br)


def relative(a, b):
    ap, ar = a
    bp, br = b
    inv = transpose(ar)
    return apply(inv, [x-y for x, y in zip(bp, ap)]), multiply(inv, br)


def origin_pose(row):
    origin = row.get('origin')
    if not isinstance(origin, dict):
        raise ValueError(f"asset {row['id']}: mounted asset requires fixed joint origin")
    return (vector([origin.get(k) for k in ('x', 'y', 'z')], 'origin XYZ'),
            rotation([origin.get(k) for k in ('roll', 'pitch', 'yaw')]))


def write_origin(row, value):
    xyz, rot = value
    row['origin'] = dict(zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz + angles(rot)))


def write_world(row, value):
    row['pose_xyz'], rot = value
    row['pose_rpy'] = angles(rot)
    row['frame'] = 'world'
    if 'pose' in row:
        row['pose'] = {'xyz': row['pose_xyz'], 'rpy': row['pose_rpy']}


def anchors(row):
    """Named datums of one asset in its own frame; every datum is a full frame.

    A plane datum lies in its local XY plane with +Z as the outward normal; an
    axis datum runs along its local +Z through its origin; a point datum is its
    origin. Untyped v1 anchors are points.
    """
    result = {'origin': {'pose_xyz': [0., 0., 0.], 'pose_rpy': [0., 0., 0.], 'datum_type': 'origin'}}
    # Reviewed support surface height is relative to the asset origin.
    if row.get('surface_z_m') is not None:
        result['surface'] = {'pose_xyz': [0., 0., row['surface_z_m']], 'pose_rpy': [0., 0., 0.], 'datum_type': 'plane'}
    supplied = row.get('mounting_anchors', {})
    if not isinstance(supplied, dict):
        raise ValueError(f"asset {row['id']}: mounting_anchors must be a named mapping")
    if 'origin' in supplied:
        raise ValueError(f"asset {row['id']}: origin anchor is reserved")
    for name, value in supplied.items():
        if not isinstance(name, str) or not name or not isinstance(value, dict):
            raise ValueError(f"asset {row['id']}: invalid anchor {name!r}")
        datum_type = value.get('datum_type', 'point')
        if datum_type not in DATUM_TYPES[1:]:
            raise ValueError(f"asset {row['id']}: datum {name!r} has unsupported type {datum_type!r}; use point, axis or plane")
        result[name] = dict(value, datum_type=datum_type)
    for name, value in result.items():
        pose(value)
    return result


def _calibration_matches(row, calibration):
    mesh = row.get('mesh') if isinstance(row.get('mesh'), dict) else {}
    defaults = {'scale': [1., 1., 1.], 'rpy': [0., 0., 0.], 'origin_offset': [0., 0., 0.]}
    for key, expected in (calibration or {}).items():
        actual = mesh.get(key, defaults.get(key))
        try:
            if len(actual) != len(expected) or any(abs(float(a)-float(b)) > 1e-9 for a, b in zip(actual, expected)):
                return False
        except (TypeError, ValueError):
            return False
    return True


def catalog_datums(row):
    """Reviewed datums for the asset's catalog entry, not yet in the scene.

    They are copied into the asset's own mounting_anchors when a mate uses
    them, so resolving a saved scene never reads the catalog. Datums are only
    offered while the asset's mesh calibration matches the reviewed one.
    """
    asset = row.get('catalog_asset_id')
    if not asset or not DATUM_CATALOG.is_file():
        return {}
    import yaml
    entries = (yaml.safe_load(DATUM_CATALOG.read_text(encoding='utf-8')) or {}).get('assets') or {}
    entry = entries.get(asset) or {}
    if not _calibration_matches(row, entry.get('calibration')):
        return {}
    own = anchors(row)
    result = {}
    for name, value in (entry.get('datums') or {}).items():
        if name in own:
            continue
        candidate = {'id': row['id'], 'mounting_anchors': {name: copy.deepcopy(value)}}
        result[name] = dict(anchors(candidate)[name], catalog_asset_id=asset)
    return result


def mate_of(row):
    mate = row.get('mate')
    if mate is None:
        return 'origin', 'coincident'
    if not isinstance(mate, dict) or set(mate) - {'child_datum', 'alignment'}:
        raise ValueError(f"asset {row['id']}: mate must be a mapping of child_datum and alignment; "
                         "one mate per asset is supported")
    alignment = mate.get('alignment', 'coincident')
    if alignment not in ALIGNMENTS:
        raise ValueError(f"asset {row['id']}: unsupported alignment {alignment!r}; use coincident or flush")
    datum = mate.get('child_datum', 'origin')
    if not isinstance(datum, str) or not datum:
        raise ValueError(f"asset {row['id']}: mate child_datum must name one of its datums")
    return datum, alignment


def check_mate(identifier, parent, link, parent_datum, child_name, child_datum, alignment):
    pt, ct = parent_datum['datum_type'], child_datum['datum_type']
    if alignment == 'flush' and (pt != 'plane' or ct != 'plane'):
        raise ValueError(f"asset {identifier}: flush needs two plane datums, but {child_name} is {ct} and "
                         f"{parent}.{link} is {pt}; choose Coincident or select plane datums")
    if {pt, ct} == {'axis', 'plane'}:
        raise ValueError(f"asset {identifier}: an axis datum cannot coincide with a plane datum "
                         f"({child_name} is {ct}, {parent}.{link} is {pt}); select matching datum types")


def inverse(a):
    p, r = a
    inv = transpose(r)
    return [-x for x in apply(inv, p)], inv


def alignment_pose(alignment):
    return [0., 0., 0.], rotation([math.pi if alignment == 'flush' else 0., 0., 0.])


def child_frame(row, child_named):
    """Child asset frame relative to its aligned datum frame: A * D_c^-1."""
    name, alignment = mate_of(row)
    return compose(alignment_pose(alignment), inverse(pose(child_named[name])))


def _index(rows):
    index = {}
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('id'), str) or not row['id']:
            raise ValueError('Every physical asset requires a stable ID')
        if row['id'] in index:
            raise ValueError(f"duplicate asset ID {row['id']!r}; rename the duplicate")
        index[row['id']] = row
        if row.get('joint_type', 'fixed') != 'fixed':
            raise ValueError(f"asset {row['id']}: only fixed mounting is supported")
        if row.get('frame', 'world') != 'world':
            raise ValueError(f"asset {row['id']}: world pose projection requires frame world")
        if isinstance(row.get('parent_object'), (list, tuple, dict)) or 'mates' in row:
            raise ValueError(f"asset {row['id']}: one parent per asset is supported; a second mate would over-constrain it")
        world_pose(row)
        anchors(row)
        mate_of(row)
        if 'mate' in row and row.get('parent_object', 'world') == 'world':
            raise ValueError(f"asset {row['id']}: a datum mate needs a parent asset; detach removes the mate")
        if 'origin' in row and row.get('parent_object', 'world') != 'world':
            offset = origin_pose(row)[0]
            if any(abs(v) > OFFSET_LIMIT_M for v in offset):
                raise ValueError(f"asset {row['id']}: mate offset must be within {OFFSET_LIMIT_M:g} m")
    return index


def resolve(rows):
    index = _index(rows)
    resolved, visiting = {}, set()

    def visit(identifier):
        if identifier in resolved:
            return resolved[identifier]
        if identifier in visiting:
            raise ValueError(f'mounting cycle at asset {identifier!r}; select a parent outside its descendants')
        visiting.add(identifier)
        row = index[identifier]
        parent = row.get('parent_object', 'world')
        link = row.get('parent_link', 'world' if parent == 'world' else 'origin')
        if parent == 'world':
            if link != 'world':
                raise ValueError(f"asset {identifier}: invalid anchor {link!r} on world; select world")
            value = world_pose(row)
        else:
            if semantic_zone(row):
                raise ValueError(f'asset {identifier}: task zones use destination bindings; mount the physical target instead')
            if parent not in index:
                raise ValueError(f"asset {identifier}: missing parent {parent!r}; restore it or detach the asset")
            if semantic_zone(index[parent]):
                raise ValueError(f'asset {identifier}: task zones cannot be mounting parents; select a physical asset')
            named = anchors(index[parent])
            if link not in named:
                raise ValueError(f"asset {identifier}: invalid anchor {link!r} on {parent}; select {', '.join(named)}")
            value = compose(compose(visit(parent), pose(named[link])), origin_pose(row))
            if 'mate' in row:
                own = anchors(row)
                child_name, alignment = mate_of(row)
                if child_name not in own:
                    raise ValueError(f"asset {identifier}: missing datum {child_name!r} on the asset itself; "
                                     f"select {', '.join(own)}")
                check_mate(identifier, parent, link, named[link], child_name, own[child_name], alignment)
                value = compose(value, child_frame(row, own))
        visiting.remove(identifier)
        resolved[identifier] = value
        return value

    for identifier in index:
        visit(identifier)
    return resolved


def validate(rows):
    resolved = resolve(rows)
    for row in rows:
        xyz, rot = world_pose(row)
        expected_xyz, expected_rot = resolved[row['id']]
        if (math.dist(xyz, expected_xyz) > 1e-7 or
                max(abs(x-y) for a, b in zip(rot, expected_rot) for x, y in zip(a, b)) > 1e-7):
            raise ValueError(f"asset {row['id']}: stale world pose; save the parent/child edit together before generating")
    return resolved


def _materialize(row, name):
    """Copy a reviewed catalog datum into the asset before a mate uses it."""
    if name in anchors(row):
        return
    supplied = catalog_datums(row).get(name)
    if supplied is None:
        return
    value = {key: copy.deepcopy(supplied[key]) for key in ('pose_xyz', 'pose_rpy', 'datum_type', 'label', 'review') if key in supplied}
    value['source'] = f"catalog:{supplied['catalog_asset_id']}"
    row.setdefault('mounting_anchors', {})[name] = value


def edit(rows, identifier, *, parent_object=None, parent_link=None,
         relative_pose=None, world_pose=None, child_datum=None, alignment=None):
    """Atomic edit; reparent/detach preserve the world pose by default.

    child_datum/alignment set the datum mate together with parent_object and
    parent_link (the parent datum). relative_pose is then the offset of the
    child datum in the parent datum frame and moves the asset; otherwise the
    world pose is kept and the offset follows from it.
    """
    before = validate(rows)
    updated = copy.deepcopy(rows)
    index = _index(updated)
    if identifier not in index:
        raise ValueError(f'missing asset {identifier!r}')
    row = index[identifier]
    mate_changed = child_datum is not None or alignment is not None
    if parent_object is not None:
        if parent_object != 'world' and parent_object not in index:
            raise ValueError(f"asset {identifier}: missing parent {parent_object!r}; restore it or detach the asset")
        row['parent_object'] = parent_object
        row['parent_link'] = parent_link or ('world' if parent_object == 'world' else 'origin')
        row['joint_type'] = 'fixed'
        if parent_object != 'world':
            _materialize(index[parent_object], row['parent_link'])
            write_origin(row, ([0., 0., 0.], rotation([0., 0., 0.])))
        else:
            row.pop('origin', None)
            row.pop('mate', None)
    if mate_changed:
        if row.get('parent_object', 'world') == 'world':
            raise ValueError(f"asset {identifier}: select a parent asset and datum before choosing a child datum")
        name = child_datum if child_datum is not None else mate_of(row)[0]
        aligned = alignment if alignment is not None else mate_of(row)[1]
        _materialize(row, name)
        if name == 'origin' and aligned == 'coincident':
            row.pop('mate', None)  # The v1 relation; keep its data format.
        else:
            row['mate'] = {'child_datum': name, 'alignment': aligned}
    if parent_object is not None or mate_changed:
        resolve(updated)  # Reject missing references, datum types and cycles before mutation.
    parent = row.get('parent_object', 'world')
    if parent == 'world':
        if relative_pose is not None:
            write_world(row, pose({'pose': relative_pose}))
        elif world_pose is not None:
            write_world(row, pose({'pose': world_pose}))
        else:
            write_world(row, before[identifier])
    else:
        link = row.get('parent_link', 'origin')
        base = compose(before[parent], pose(anchors(index[parent])[link]))
        if relative_pose is not None:
            write_origin(row, pose({'pose': relative_pose}))
        elif world_pose is not None or parent_object is not None or mate_changed:
            target = pose({'pose': world_pose}) if world_pose else before[identifier]
            if 'mate' in row:
                # The offset is between datum frames: undo A * D_c^-1.
                target = compose(target, inverse(child_frame(row, anchors(row))))
            write_origin(row, relative(base, target))
    resolved = resolve(updated)
    for item in updated:
        write_world(item, resolved[item['id']])
    validate(updated)
    return updated


def datum_frames(rows):
    """World frames of every datum, for visual mate picking.

    The resolver owns the transform: each asset's resolved world pose composed
    with its datum frame (own anchors, the reviewed support surface and, while
    the mesh calibration matches, reviewed catalog datums). Viewports only
    compare picked geometry against these frames; task zones are not physical
    mounting targets and are omitted.
    """
    resolved = validate(rows)
    result = {}
    for row in rows:
        if semantic_zone(row):
            continue
        world = resolved[row['id']]
        own = anchors(row)
        frames = {}
        for name, datum in {**catalog_datums(row), **own}.items():
            xyz, rot = compose(world, pose(datum))
            if name == 'origin':
                source = 'asset origin'
            elif name == 'surface' and 'surface' not in (row.get('mounting_anchors') or {}):
                source = 'reviewed support surface (surface_z_m)'
            elif 'catalog_asset_id' in datum:
                source = f"reviewed catalog datum ({datum['catalog_asset_id']}), copied into the cell when used"
            else:
                source = datum.get('source') or 'asset mounting anchor'
            frames[name] = {
                'datum_type': datum['datum_type'], 'label': datum.get('label', ''), 'source': source,
                'xyz': xyz, 'rpy': angles(rot),
                'x_axis': [rot[0][0], rot[1][0], rot[2][0]], 'normal': [rot[0][2], rot[1][2], rot[2][2]]}
        xyz, rot = world
        result[row['id']] = {'world': {'xyz': xyz, 'rpy': angles(rot)}, 'datums': frames}
    return result


def physical_rows(physical):
    return [item for key in COLLECTIONS for item in physical.get(key, [])]


def validate_physical(physical):
    return validate(physical_rows(physical))


def saved_mesh_calibration(scene_dir):
    """Mesh mappings of the saved layout: the editor keeps them unchanged on Save."""
    path = Path(scene_dir) / 'layout' / 'workcell_studio_layout.yaml'
    if not scene_dir or not path.is_file():
        return {}
    import yaml
    try:
        items = (yaml.safe_load(path.read_text(encoding='utf-8')) or {}).get('items') or []
    except (OSError, yaml.YAMLError, AttributeError):
        return {}  # No reviewed datums are offered without a readable calibration.
    return {item['id']: item['mesh'] for item in items
            if isinstance(item, dict) and isinstance(item.get('mesh'), dict) and item.get('id')}


if __name__ == '__main__':
    try:
        request = json.load(sys.stdin)
        rows = request['items']
        scene_dir = request.pop('scene_dir', '')
        saved = saved_mesh_calibration(scene_dir)
        for row in rows:
            metadata = row.pop('mounting_yaml', '')
            if metadata:
                import yaml
                row.update(yaml.safe_load(metadata) or {})
            if row.get('catalog_asset_id') and row['id'] in saved:
                row['mesh'] = saved[row['id']]
        if request.get('operation') == 'anchors':
            validate(rows)
            result = {row['id']: {**catalog_datums(row), **anchors(row)} for row in rows}
        elif request.get('operation') == 'datum_frames':
            result = datum_frames(rows)
        elif request.get('operation') == 'align':
            # Two-part Align & Arrange: plans an ordinary edit request (asset_alignment.py).
            from asset_alignment import plan
            result = plan(rows, request, scene_dir)
            for row in result['rows']:
                row['mounting_yaml'] = json.dumps({key: row[key] for key in (*RELATION_FIELDS, 'surface_z_m') if key in row})
        else:
            result = edit(rows, request.pop('id'), **{
                key: value for key, value in request.items()
                if key in ('parent_object', 'parent_link', 'relative_pose', 'world_pose', 'child_datum', 'alignment')})
        if isinstance(result, list):
            for row in result:
                row['mounting_yaml'] = json.dumps({key: row[key] for key in (*RELATION_FIELDS, 'surface_z_m') if key in row})
        print(json.dumps({'ok': True, 'result': result}, allow_nan=False))
    except (ValueError, KeyError, TypeError) as error:
        print(json.dumps({'ok': False, 'error': str(error)}))
        sys.exit(2)
