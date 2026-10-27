#!/usr/bin/env python3
"""Two-part Align & Arrange on the existing mounting contract.

One part (the moving part) is placed relative to another (the reference) by
the bounding boxes of their authored geometry. Every rule is expressed in the
reference part's own frame, so a rotated reference keeps its own left, right,
front, back, top and bottom:

    left/right = -X/+X, front/back = -Y/+Y, bottom/top = -Z/+Z

Each part's box is the axis-aligned box of its real geometry in its own frame:
mesh vertices after the authored mesh calibration (scale, then mesh RPY, then
origin_offset, the URDF visual-origin convention the Product View and the
collision manifest use), or the authored dimensions of a box primitive. The
moving part's box is projected into the reference frame. Nothing is guessed:
a part without readable geometry is rejected with the reason.

Only "Place on top" may use a verified datum: the reference's reviewed upward
plane datum (for example a workbench `surface`). Every other result is a
bounding-box alignment, not a CAD face mate.

The result is an ordinary asset_mounting edit request: a world pose that keeps
the part's orientation, and for "keep relative" a fixed mount on the reference.
asset_mounting.edit applies it, so mounted children follow and a mounted part
keeps its parent, exactly as for every other placement edit.
"""
from __future__ import annotations

import math
from pathlib import Path

import asset_mounting as mounting
from physical_destination import angles, apply, multiply, rotation, transpose

AXES = ('x', 'y', 'z')
RULES = {'x': ('centre', 'min', 'max'), 'y': ('centre', 'min', 'max'), 'z': ('centre', 'min', 'max', 'on_top')}
LAYOUT = Path('layout') / 'workcell_studio_layout.yaml'
MAX_EXTENT_M = 50.  # larger parts are almost always millimetre meshes without scale
MAX_OFFSET_M = 10.
PERMUTATION_TOLERANCE = 1e-6
POSITION_DECIMALS = 9  # 1 nm


class AlignmentError(ValueError):
    """A rejected alignment; the message names the part and the next step."""


def _vector(value, what, positive=False):
    if (not isinstance(value, (list, tuple)) or len(value) != 3 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise AlignmentError(f'{what} requires three finite numbers')
    if positive and any(v <= 0 for v in value):
        raise AlignmentError(f'{what} must be positive, got {list(value)}')
    return [float(v) for v in value]


def saved_layout_items(scene_dir):
    path = Path(scene_dir or '') / LAYOUT
    if not scene_dir or not path.is_file():
        raise AlignmentError(f'the saved layout {path} is missing; save the cell before aligning')
    import yaml
    try:
        items = (yaml.safe_load(path.read_text(encoding='utf-8')) or {}).get('items') or []
    except (OSError, yaml.YAMLError, AttributeError) as error:
        raise AlignmentError(f'the saved layout {path} cannot be read ({error})') from error
    return {item['id']: item for item in items if isinstance(item, dict) and item.get('id')}


def _stl_points(path):
    import numpy as np
    data = path.read_bytes()
    if len(data) >= 84:
        count = int(np.frombuffer(data, dtype='<u4', count=1, offset=80)[0])
        if count and 84 + 50 * count == len(data):
            record = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
            return np.frombuffer(data, dtype=record, count=count, offset=84)['v'].reshape(-1, 3).astype(float), count
    import re
    values = re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data)
    points = np.array(values, dtype=float) if values else np.zeros((0, 3))
    return points, len(points) // 3


def _mesh_file(reference, scene_dir):
    from generate_moveit_collision_manifest import _resolve_local_mesh
    candidate = _resolve_local_mesh(reference, Path(scene_dir) / 'layout')
    if candidate is None and isinstance(reference, str) and not reference.startswith('package://'):
        candidate = _resolve_local_mesh(reference, Path(scene_dir))
    return candidate


def _box_size(item):
    if item.get('dimensions') is None and isinstance(item.get('size'), dict):  # the editor's width/depth/height form
        return [item['size'].get(key) for key in ('width', 'depth', 'height')]
    return item.get('dimensions')


def part_box(item, scene_dir, name):
    """(lo, hi, source) of the part's geometry in its own frame."""
    mesh = item.get('mesh') if isinstance(item.get('mesh'), dict) else {}
    reference = mesh.get('path') or mesh.get('uri')
    geometry = str(item.get('geometry_type') or ('mesh' if reference else '')).lower()
    if geometry == 'mesh':
        if not reference:
            raise AlignmentError(f'{name} is a mesh part without a mesh path; it has no usable geometry')
        path = _mesh_file(reference, scene_dir)
        if path is None:
            raise AlignmentError(f'{name}: mesh {reference} cannot be found, so its size is unknown; '
                                 'restore the mesh or choose another part')
        if path.suffix.lower() != '.stl':
            raise AlignmentError(f'{name}: only STL mesh bounds are read for alignment ({path.name} is '
                                 f'{path.suffix or "untyped"}); use Mate with its verified datums instead')
        import numpy as np
        points, triangles = _stl_points(path)
        if not len(points) or not np.isfinite(points).all():
            raise AlignmentError(f'{name}: mesh {path.name} has no readable finite vertices')
        scale = _vector(mesh.get('scale', [1., 1., 1.]), f'{name} mesh scale', positive=True)
        rot = rotation(_vector(mesh.get('rpy', [0., 0., 0.]), f'{name} mesh rpy'))
        offset = _vector(mesh.get('origin_offset', [0., 0., 0.]), f'{name} mesh origin_offset')
        local = (points * np.array(scale)) @ np.array(rot).T + np.array(offset)
        lo, hi = local.min(axis=0).tolist(), local.max(axis=0).tolist()
        source = f'mesh vertices ({path.name}, {triangles} triangles, authored scale/RPY/offset)'
    elif geometry == 'box':
        size = _vector(_box_size(item), f'{name} box dimensions', positive=True)
        lo, hi = [-v / 2 for v in size], [v / 2 for v in size]
        source = 'authored box dimensions'
    else:
        raise AlignmentError(f'{name} has {geometry or "no"} geometry; alignment reads STL meshes and boxes only')
    extent = [b - a for a, b in zip(lo, hi)]
    if max(extent) > MAX_EXTENT_M:
        raise AlignmentError(f'{name} measures {max(extent):.1f} m across; check its mesh units '
                             '(a millimetre STL needs scale 0.001)')
    if max(extent) <= 1e-6:
        raise AlignmentError(f'{name} has degenerate geometry (all extents are zero)')
    return lo, hi, source


def _corners(lo, hi):
    return [[(lo, hi)[i >> k & 1][k] for k in range(3)] for i in range(8)]


def _axis_aligned(rot):
    return all(min(abs(abs(v) - target) for target in (0., 1.)) < PERMUTATION_TOLERANCE for row in rot for v in row)


def verified_top(row, catalog):
    """Height of the reference's reviewed upward plane datum in its own frame, if any."""
    named = {**catalog, **mounting.anchors(row)}
    for name in ('surface', *sorted(named)):
        datum = named.get(name)
        if not datum or datum.get('datum_type') != 'plane':
            continue
        xyz, rot = mounting.pose(datum)
        normal = [rot[0][2], rot[1][2], rot[2][2]]
        if normal[2] > 1. - 1e-9:
            return xyz[2], name
    return None, ''


def descendants(rows, identifier):
    children = {}
    for row in rows:
        children.setdefault(row.get('parent_object', 'world'), []).append(row['id'])
    found, stack = set(), [identifier]
    while stack:
        for child in children.get(stack.pop(), []):
            if child not in found:
                found.add(child)
                stack.append(child)
    return found


def plan(rows, request, scene_dir):
    """The mounting edit request and a report for one alignment, without applying it."""
    moving, reference = request.get('id'), request.get('reference')
    names = request.get('names') or {}
    name = lambda identifier: names.get(identifier) or identifier  # noqa: E731
    if not moving or not reference:
        raise AlignmentError('choose the part to move and a reference part')
    if moving == reference:
        raise AlignmentError(f'{name(moving)} cannot be aligned to itself; choose a different reference part')
    index = {row['id']: row for row in rows}
    if moving not in index:
        raise AlignmentError(f'{name(moving)} is locked or not an editable placed asset, so it cannot be moved')
    for identifier, role in ((moving, 'moved'), (reference, 'used as a reference')):
        if identifier in index and mounting.semantic_zone(index[identifier]):
            raise AlignmentError(f'{name(identifier)} is a task zone; zones are task semantics, not physical parts, '
                                 f'and cannot be {role}')
    if reference in descendants(rows, moving):
        raise AlignmentError(f'{name(reference)} is mounted on {name(moving)} and moves with it; '
                             'choose a reference that does not follow the moving part')
    rules = request.get('rules') or {}
    for axis, rule in rules.items():
        if axis not in RULES or (rule is not None and rule not in RULES[axis]):
            raise AlignmentError(f'unsupported rule {rule!r} for axis {axis!r}')
    active = [axis for axis in AXES if rules.get(axis)]
    if not active:
        raise AlignmentError('choose at least one Centre, Align or Place operation')
    offset = _vector(request.get('offset_m', [0., 0., 0.]), 'offset')
    if any(abs(v) > MAX_OFFSET_M for v in offset):
        raise AlignmentError(f'offsets must be within {MAX_OFFSET_M:g} m')

    resolved = mounting.validate(rows)
    saved = saved_layout_items(scene_dir)
    for identifier in (moving, reference):
        if identifier not in saved:
            raise AlignmentError(f'{name(identifier)} is not in the saved layout yet, so its geometry is not on disk; '
                                 'Save Layout, then align')
    live = request.get('live_dimensions') or {}
    for identifier, dims in live.items():
        authored = _box_size(saved.get(identifier, {}))
        if identifier in (moving, reference) and authored is not None and dims is not None:
            if any(not isinstance(a, (int, float)) or abs(float(a) - float(b)) > 1e-9 for a, b in zip(authored, dims)):
                raise AlignmentError(f'{name(identifier)} has unsaved dimension edits; Save Layout so alignment uses '
                                     'the size you see')

    if reference in resolved:
        ref_xyz, ref_rot = resolved[reference]
    else:
        world = request.get('reference_world') or {}
        ref_xyz = _vector(world.get('xyz'), f'{name(reference)} world position')
        ref_rot = rotation(_vector(world.get('rpy'), f'{name(reference)} world rotation'))
    mov_xyz, mov_rot = resolved[moving]

    mov_lo, mov_hi, mov_source = part_box(saved[moving], scene_dir, name(moving))
    ref_lo, ref_hi, ref_source = part_box(saved[reference], scene_dir, name(reference))
    to_ref = transpose(ref_rot)
    projected = [apply(to_ref, [p + q - r for p, q, r in zip(mov_xyz, apply(mov_rot, corner), ref_xyz)])
                 for corner in _corners(mov_lo, mov_hi)]
    lo = [min(c[k] for c in projected) for k in range(3)]
    hi = [max(c[k] for c in projected) for k in range(3)]

    notes, sources = [], {'moving': mov_source, 'reference': ref_source}
    delta = [0., 0., 0.]
    for k, axis in enumerate(AXES):
        rule = rules.get(axis)
        if not rule:
            continue
        if rule == 'centre':
            delta[k] = (ref_lo[k] + ref_hi[k]) / 2 - (lo[k] + hi[k]) / 2
        elif rule == 'min':
            delta[k] = ref_lo[k] - lo[k]
        elif rule == 'max':
            delta[k] = ref_hi[k] - hi[k]
        else:
            top, datum = None, ''
            if reference in index:
                row = dict(index[reference], mesh=saved[reference].get('mesh'))
                top, datum = verified_top(row, mounting.catalog_datums(row))
            if top is None:
                top = ref_hi[2]
                sources['top'] = 'bounding-box top of the reference'
            else:
                sources['top'] = f"verified surface datum '{datum}' of the reference"
            delta[k] = top - lo[k]
        delta[k] += offset[k]
    if not _axis_aligned(multiply(to_ref, mov_rot)):
        notes.append(f'{name(moving)} is rotated relative to {name(reference)}: its box in the reference frame '
                     'encloses the rotated box, so edges can look loose')

    world_delta = apply(ref_rot, delta)
    # Nanometre rounding keeps float noise (0.5599999999999442) out of the saved layout.
    target_xyz = [round(a + b, POSITION_DECIMALS) + 0. for a, b in zip(mov_xyz, world_delta)]
    world_delta = [t - a for t, a in zip(target_xyz, mov_xyz)]
    target = {'xyz': target_xyz, 'rpy': angles(mov_rot)}
    edit = {'id': moving, 'world_pose': target}
    keep = bool(request.get('keep_relative'))
    current_parent = index[moving].get('parent_object', 'world')
    if keep:
        if reference not in index:
            raise AlignmentError(f'{name(reference)} is locked or generated, so {name(moving)} cannot follow it; '
                                 'use Align once')
        edit.update(parent_object=reference, parent_link='origin', child_datum='origin', alignment='coincident')
        if current_parent not in ('world', reference):
            notes.append(f'{name(moving)} is now mounted on {name(current_parent)}; keeping it relative to '
                         f'{name(reference)} replaces that mount')
    elif current_parent != 'world':
        notes.append(f'{name(moving)} stays mounted on {name(current_parent)}; only its offset changes')
    after = mounting.edit(rows, moving, **{key: value for key, value in edit.items() if key != 'id'})
    return {
        'request': edit,
        'rows': after,
        'report': {
            'moving': moving, 'reference': reference, 'rules': {axis: rules.get(axis) for axis in AXES},
            'offset_m': offset, 'keep_relative': keep, 'frame': f'{reference} (reference part frame)',
            'delta_reference_m': delta, 'delta_world_m': world_delta,
            'distance_m': math.sqrt(sum(v * v for v in world_delta)),
            'current_world': {'xyz': mov_xyz, 'rpy': angles(mov_rot)}, 'target_world': target,
            'moving_box_reference_m': {'min': lo, 'max': hi},
            'reference_box_m': {'min': ref_lo, 'max': ref_hi},
            'method': 'bounding_box', 'sources': sources, 'notes': notes,
        },
    }
