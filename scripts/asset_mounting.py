#!/usr/bin/env python3
"""Fixed asset joints in the existing physical scene contract.

parent_object/parent_link/origin own a mounted pose; pose_xyz/pose_rpy are
checked world projections for existing scene, collision and destination readers.
No robot/tool joints or mesh calibration transforms are changed here.
"""
from __future__ import annotations
import copy
import json
import math
import sys

from physical_destination import angles, apply, multiply, pose, rotation, transpose, vector


def world_pose(row):
    value = row.get("pose", {})
    return pose({"pose_xyz": row.get("pose_xyz", value.get("xyz", [value.get(k, 0.) for k in ("x", "y", "z")])),
                 "pose_rpy": row.get("pose_rpy", value.get("rpy", [value.get(k, 0.) for k in ("roll", "pitch", "yaw")]))})

RELATION_FIELDS = ('parent_object', 'parent_link', 'joint_type', 'origin', 'mounting_anchors')
COLLECTIONS = ('support_surfaces', 'assets', 'sensors', 'task_zones')


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
    result = {'origin': {'pose_xyz': [0., 0., 0.], 'pose_rpy': [0., 0., 0.]}}
    # Reviewed support surface height is relative to the asset origin.
    if row.get('surface_z_m') is not None:
        result['surface'] = {'pose_xyz': [0., 0., row['surface_z_m']], 'pose_rpy': [0., 0., 0.]}
    supplied = row.get('mounting_anchors', {})
    if not isinstance(supplied, dict):
        raise ValueError(f"asset {row['id']}: mounting_anchors must be a named mapping")
    if 'origin' in supplied:
        raise ValueError(f"asset {row['id']}: origin anchor is reserved")
    result.update(supplied)
    for name, value in result.items():
        if not isinstance(name, str) or not name or not isinstance(value, dict):
            raise ValueError(f"asset {row['id']}: invalid anchor {name!r}")
        pose(value)
    return result


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
        world_pose(row)
        anchors(row)
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


def edit(rows, identifier, *, parent_object=None, parent_link=None,
         relative_pose=None, world_pose=None):
    """Atomic edit; reparent/detach preserve the world pose by default."""
    before = validate(rows)
    updated = copy.deepcopy(rows)
    index = _index(updated)
    if identifier not in index:
        raise ValueError(f'missing asset {identifier!r}')
    row = index[identifier]
    if parent_object is not None:
        row['parent_object'] = parent_object
        row['parent_link'] = parent_link or ('world' if parent_object == 'world' else 'origin')
        row['joint_type'] = 'fixed'
        if parent_object != 'world':
            write_origin(row, ([0., 0., 0.], rotation([0., 0., 0.])))
        else:
            row.pop('origin', None)
        resolve(updated)  # Reject missing references/cycles before mutation.
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
        elif world_pose is not None or parent_object is not None:
            write_origin(row, relative(base, pose({'pose': world_pose}) if world_pose else before[identifier]))
    resolved = resolve(updated)
    for item in updated:
        write_world(item, resolved[item['id']])
    validate(updated)
    return updated


def physical_rows(physical):
    return [item for key in COLLECTIONS for item in physical.get(key, [])]


def validate_physical(physical):
    return validate(physical_rows(physical))


if __name__ == '__main__':
    try:
        request = json.load(sys.stdin)
        rows = request['items']
        for row in rows:
            metadata = row.pop('mounting_yaml', '')
            if metadata:
                import yaml
                row.update(yaml.safe_load(metadata) or {})
        if request.get('operation') == 'anchors':
            validate(rows)
            result = {row['id']: anchors(row) for row in rows}
        else:
            result = edit(rows, request.pop('id'), **{
                key: value for key, value in request.items()
                if key in ('parent_object', 'parent_link', 'relative_pose', 'world_pose')})
        if isinstance(result, list):
            for row in result:
                row['mounting_yaml'] = json.dumps({key: row[key] for key in (*RELATION_FIELDS, 'surface_z_m') if key in row})
        print(json.dumps({'ok': True, 'result': result}, allow_nan=False))
    except (ValueError, KeyError, TypeError) as error:
        print(json.dumps({'ok': False, 'error': str(error)}))
        sys.exit(2)
