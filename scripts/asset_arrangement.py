#!/usr/bin/env python3
"""Multi-object arrangement: one atomic edit for a selection and its reference.

The selection is a reference part (which never moves) plus one or more members.
Every operation produces ordinary asset_mounting edits, applied in sequence to one
copy of the rows, so the result is a single authoring transaction:

* align       - asset_alignment rules for every member against the reference
* distribute  - equal spacing or an exact gap along one reference axis
* move        - precise translation/rotation in world, reference or local axes
* snap_grid   - member origins snapped to a horizontal world grid
* mount       - fixed mount of every member on the reference (world pose kept)
* detach      - mounted members become world-placed (world pose kept)

Boxes are the authored-geometry boxes of asset_alignment (never a default size).
Results are deterministic: members are processed in a stable order and ties along
an axis are broken by ID. Locked members, task zones, members mounted on other
members, and references that follow a member are refused before anything moves.
"""
from __future__ import annotations

import math

import asset_alignment as alignment
import asset_mounting as mounting
from physical_destination import angles, apply, multiply, rotation, transpose

OPERATIONS = ('align', 'distribute', 'move', 'snap_grid', 'mount', 'detach')
AXES = ('x', 'y', 'z')
FRAMES = ('world', 'reference', 'local')
MAX_MOVE_M = 10.
DECIMALS = alignment.POSITION_DECIMALS


class ArrangementError(alignment.AlignmentError):
    """A refused arrangement; the message names the parts and the next step."""


def _round(xyz):
    return [round(v, DECIMALS) + 0. for v in xyz]


def _rpy_rotation(rpy_deg):
    return rotation([math.radians(float(v)) for v in rpy_deg])


def _members(rows, request, name):
    reference = request.get('reference')
    members = []
    for identifier in request.get('members') or []:
        if identifier and identifier != reference and identifier not in members:
            members.append(identifier)
    if not reference:
        raise ArrangementError('choose a reference part (the first selected part, or Make Reference)')
    if not members:
        raise ArrangementError('select at least one part besides the reference (Ctrl+click to add parts)')
    index = {row['id']: row for row in rows}
    for identifier in members:
        if identifier not in index:
            raise ArrangementError(f'{name(identifier)} is locked or not an editable placed asset, so it cannot move; '
                                   'make it the reference or remove it from the selection')
        if mounting.semantic_zone(index[identifier]):
            raise ArrangementError(f'{name(identifier)} is a task zone; zones follow their target and cannot be arranged')
    if reference in index and mounting.semantic_zone(index[reference]):
        raise ArrangementError(f'{name(reference)} is a task zone, not a physical reference part')
    return reference, members, index


def _check_hierarchy(rows, reference, members, name, moving=True):
    if not moving:
        return
    for identifier in members:
        followers = alignment.descendants(rows, identifier)
        if reference in followers:
            raise ArrangementError(f'{name(reference)} is mounted on {name(identifier)} and would move with it; '
                                   'choose another reference')
        for other in members:
            if other in followers:
                raise ArrangementError(f'{name(other)} is mounted on {name(identifier)}, so it already follows it; '
                                       f'remove {name(other)} from the selection or detach it first')


def _reference_pose(rows, request, reference, name):
    resolved = mounting.validate(rows)
    if reference in resolved:
        return resolved[reference]
    world = request.get('reference_world') or {}
    xyz = alignment._vector(world.get('xyz'), f'{name(reference)} world position')
    return xyz, rotation(alignment._vector(world.get('rpy'), f'{name(reference)} world rotation'))


def _interval(identifier, rows_pose, saved, scene_dir, name, ref_xyz, ref_rot, axis):
    """[lo, hi] of a part's authored-geometry box along one reference axis."""
    xyz, rot = rows_pose
    lo, hi, source = alignment.part_box(saved[identifier], scene_dir, name(identifier))
    to_ref = transpose(ref_rot)
    values = [apply(to_ref, [p + q - r for p, q, r in zip(xyz, apply(rot, corner), ref_xyz)])[axis]
              for corner in alignment._corners(lo, hi)]
    return min(values), max(values), source


def _edit_world(rows, identifier, xyz, rot):
    return mounting.edit(rows, identifier, world_pose={'xyz': _round(xyz), 'rpy': angles(rot)})


def _distribute(rows, request, reference, members, name, scene_dir):
    params = request.get('params') or {}
    axis_name = params.get('axis')
    if axis_name not in AXES:
        raise ArrangementError('choose the distribution axis: X, Y or Z')
    axis = AXES.index(axis_name)
    mode = params.get('mode', 'equal')
    if mode not in ('equal', 'gap'):
        raise ArrangementError(f'unsupported distribution {mode!r}; use equal spacing or a gap')
    if mode == 'equal' and len(members) < 2:
        raise ArrangementError('equal spacing needs at least three parts (the reference and two more)')
    saved = alignment.saved_layout_items(scene_dir)
    for identifier in [reference, *members]:
        if identifier not in saved:
            raise ArrangementError(f'{name(identifier)} is not in the saved layout yet; Save Layout, then arrange')
    ref_xyz, ref_rot = _reference_pose(rows, request, reference, name)
    resolved = mounting.validate(rows)
    poses = {identifier: resolved[identifier] for identifier in members}
    poses[reference] = (ref_xyz, ref_rot)
    entries, sources = [], {}
    for identifier in [reference, *members]:
        lo, hi, source = _interval(identifier, poses[identifier], saved, scene_dir, name, ref_xyz, ref_rot, axis)
        entries.append({'id': identifier, 'lo': lo, 'hi': hi})
        sources[identifier] = source
    entries.sort(key=lambda entry: ((entry['lo'] + entry['hi']) / 2, entry['id']))
    sizes = [entry['hi'] - entry['lo'] for entry in entries]
    if mode == 'equal':
        span = entries[-1]['hi'] - entries[0]['lo']
        gap = (span - sum(sizes)) / (len(entries) - 1)
        if gap < -1e-9:
            raise ArrangementError(f'the parts are {-gap * 1000:.1f} mm too long for equal spacing between the outermost '
                                   'parts; use an exact gap instead')
    else:
        gap = alignment._vector([params.get('gap_m', 0.), 0., 0.], 'gap')[0]
        if abs(gap) > MAX_MOVE_M:
            raise ArrangementError(f'the gap must be within {MAX_MOVE_M:g} m')
    r = next(i for i, entry in enumerate(entries) if entry['id'] == reference)
    new_lo = [0.] * len(entries)
    new_lo[r] = entries[r]['lo']
    for i in range(r + 1, len(entries)):
        new_lo[i] = new_lo[i - 1] + sizes[i - 1] + gap
    for i in range(r - 1, -1, -1):
        new_lo[i] = new_lo[i + 1] - gap - sizes[i]
    direction = [ref_rot[0][axis], ref_rot[1][axis], ref_rot[2][axis]]
    updated, moves = rows, {}
    for i, entry in enumerate(entries):
        if entry['id'] == reference:
            continue
        shift = new_lo[i] - entry['lo']
        xyz, rot = poses[entry['id']]
        updated = _edit_world(updated, entry['id'], [p + shift * d for p, d in zip(xyz, direction)], rot)
        moves[entry['id']] = shift
    return updated, {'axis': axis_name, 'mode': mode, 'gap_m': gap, 'order': [entry['id'] for entry in entries],
                     'shift_along_axis_m': moves, 'sources': sources}


def _move(rows, request, reference, members, name):
    params = request.get('params') or {}
    frame = params.get('frame', 'world')
    pivot = params.get('pivot', 'each')
    if frame not in FRAMES:
        raise ArrangementError(f'unsupported frame {frame!r}; use world, reference or local')
    if pivot not in ('each', 'reference'):
        raise ArrangementError(f'unsupported pivot {pivot!r}; rotate about each part or about the reference')
    translation = alignment._vector(params.get('translate_m', [0., 0., 0.]), 'translation')
    turn = alignment._vector(params.get('rotate_deg', [0., 0., 0.]), 'rotation')
    if any(abs(v) > MAX_MOVE_M for v in translation):
        raise ArrangementError(f'moves must be within {MAX_MOVE_M:g} m')
    if all(abs(v) < 1e-12 for v in translation + turn):
        raise ArrangementError('enter a distance or an angle')
    ref_xyz, ref_rot = _reference_pose(rows, request, reference, name)
    resolved = mounting.validate(rows)
    delta = _rpy_rotation(turn)
    updated = rows
    for identifier in sorted(members):
        xyz, rot = resolved[identifier]
        axes = {'world': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], 'reference': ref_rot, 'local': rot}[frame]
        world_turn = multiply(multiply(axes, delta), transpose(axes))
        if pivot == 'reference':
            xyz = [c + v for c, v in zip(ref_xyz, apply(world_turn, [p - c for p, c in zip(xyz, ref_xyz)]))]
        xyz = [p + v for p, v in zip(xyz, apply(axes, translation))]
        updated = _edit_world(updated, identifier, xyz, multiply(world_turn, rot))
    return updated, {'frame': frame, 'pivot': pivot, 'translate_m': translation, 'rotate_deg': turn}


def _snap_grid(rows, request, members):
    step = alignment._vector([(request.get('params') or {}).get('step_m', 0.), 1., 1.], 'grid step')[0]
    if step < 1e-4 or step > 1.:
        raise ArrangementError('the grid step must be between 0.1 mm and 1000 mm')
    resolved = mounting.validate(rows)
    updated = rows
    for identifier in sorted(members):
        xyz, rot = resolved[identifier]
        snapped = [round(xyz[0] / step) * step, round(xyz[1] / step) * step, xyz[2]]
        updated = _edit_world(updated, identifier, snapped, rot)
    return updated, {'step_m': step, 'snapped_axes': ['x', 'y'], 'snapped_point': 'part origin'}


def _mount(rows, reference, members, index, name):
    if reference not in index:
        raise ArrangementError(f'{name(reference)} is locked or generated, so parts cannot be mounted on it')
    updated, notes = rows, []
    for identifier in sorted(members):
        current = index[identifier].get('parent_object', 'world')
        if current == reference:
            notes.append(f'{name(identifier)} is already mounted on {name(reference)}')
            continue
        if current != 'world':
            notes.append(f'{name(identifier)} was mounted on {name(current)}; it is now mounted on {name(reference)}')
        updated = mounting.edit(updated, identifier, parent_object=reference, parent_link='origin',
                                child_datum='origin', alignment='coincident')
    if updated is rows:
        raise ArrangementError(f'every selected part is already mounted on {name(reference)}')
    return updated, {'parent': reference, 'notes': notes}


def _detach(rows, reference, members, index, name):
    updated, detached = rows, []
    for identifier in sorted([*members, reference]):
        if identifier in index and index[identifier].get('parent_object', 'world') != 'world':
            updated = mounting.edit(updated, identifier, parent_object='world')
            detached.append(identifier)
    if not detached:
        raise ArrangementError('none of the selected parts is mounted; nothing to detach')
    return updated, {'detached': detached}


def plan(rows, request, scene_dir):
    """The final rows and a report for one arrangement, without applying it."""
    names = request.get('names') or {}
    name = lambda identifier: names.get(identifier) or identifier  # noqa: E731
    op = request.get('op')
    if op not in OPERATIONS:
        raise ArrangementError(f'unsupported arrangement {op!r}')
    reference, members, index = _members(rows, request, name)
    _check_hierarchy(rows, reference, members, name, moving=op not in ('mount', 'detach'))
    before = {identifier: value for identifier, value in mounting.validate(rows).items()}
    report = {'op': op, 'reference': reference, 'members': members, 'notes': []}
    if op == 'align':
        params = request.get('params') or {}
        updated = rows
        per_member = {}
        for identifier in sorted(members):
            single = alignment.plan(updated, {'id': identifier, 'reference': reference, 'rules': params.get('rules'),
                                              'offset_m': params.get('offset_m', [0., 0., 0.]), 'names': names,
                                              'reference_world': request.get('reference_world')}, scene_dir)
            updated = single['rows']
            per_member[identifier] = {key: single['report'][key] for key in ('sources', 'notes', 'delta_reference_m')}
            report['notes'] += single['report']['notes']
        report['rules'] = params.get('rules')
        report['members_detail'] = per_member
    elif op == 'distribute':
        updated, detail = _distribute(rows, request, reference, members, name, scene_dir)
        report.update(detail)
    elif op == 'move':
        updated, detail = _move(rows, request, reference, members, name)
        report.update(detail)
    elif op == 'snap_grid':
        updated, detail = _snap_grid(rows, request, members)
        report.update(detail)
    elif op == 'mount':
        updated, detail = _mount(rows, reference, members, index, name)
        report['notes'] += detail.pop('notes')
        report.update(detail)
    else:
        updated, detail = _detach(rows, reference, members, index, name)
        report.update(detail)
    after = mounting.validate(updated)
    moved = []
    for identifier in sorted(after):
        old_xyz, old_rot = before[identifier]
        xyz, rot = after[identifier]
        distance = math.dist(old_xyz, xyz)
        turned = max(abs(a - b) for row_a, row_b in zip(old_rot, rot) for a, b in zip(row_a, row_b))
        if distance > 1e-9 or turned > 1e-9:
            moved.append({'id': identifier, 'distance_m': distance, 'from': old_xyz, 'to': xyz,
                          'follower': identifier not in members})
    if reference in {entry['id'] for entry in moved}:
        raise ArrangementError(f'{name(reference)} would move; the reference never moves')
    report['moved'] = moved
    return {'rows': updated, 'report': report}
