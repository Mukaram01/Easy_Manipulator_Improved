"""One physical destination contract; metres, radians and ROS fixed-axis RPY.

Usable boxes are reviewed free space in the physical asset frame (after mesh
origin/scale calibration), never mesh bounding boxes. World zone poses are
checked projections, not an alternative destination. Motion feasibility is
established separately by the complete collision-aware cycle planner.
"""
import itertools
import math


def vector(value, name, size=3):
    if (not isinstance(value, (list, tuple)) or len(value) != size or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in value)):
        raise ValueError(f'{name} requires {size} finite numbers')
    return list(value)


def rotation(rpy):
    r, p, y = vector(rpy, 'pose_rpy')
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def transpose(a):
    return list(map(list, zip(*a)))


def apply(a, v):
    return [sum(x*y for x, y in zip(row, v)) for row in a]


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def angles(a):
    p = math.asin(max(-1., min(1., -a[2][0])))
    if abs(math.cos(p)) < 1e-9:
        return [0., p, math.atan2(-a[0][1], a[1][1])]
    return [math.atan2(a[2][1], a[2][2]), p, math.atan2(a[1][0], a[0][0])]


def pose(block):
    value = block.get('pose', {})
    return (vector(block.get('pose_xyz', value.get('xyz')), 'pose_xyz'),
            rotation(block.get('pose_rpy', value.get('rpy'))))


def dimensions(block):
    result = vector(block.get('dimensions'), 'dimensions')
    if any(v <= 0 for v in result):
        raise ValueError('dimensions must be positive')
    return result


def contains(outer, center, orient, dims, clearance=0.):
    origin, rot = pose(outer)
    bound = dimensions(outer)
    inv = transpose(rot)
    for signs in itertools.product((-1, 1), repeat=3):
        offset = apply(orient, [s*d/2 for s, d in zip(signs, dims)])
        local = apply(inv, [c+o-b for c, o, b in zip(center, offset, origin)])
        if any(abs(v)+clearance > d/2+1e-8 for v, d in zip(local, bound)):
            raise ValueError('placement/object outside target usable placement geometry')


def relative_placement(target, world_zone):
    origin, rot = pose(target)
    center, orient = pose(world_zone)
    return {'pose_xyz': apply(transpose(rot), [a-b for a, b in zip(center, origin)]),
            'pose_rpy': angles(multiply(transpose(rot), orient)),
            'dimensions': dimensions(world_zone)}


def resolve_destination(environment, zone_id, *, check_projection=True):
    zones = [z for z in environment.get('task_zones', []) if z.get('id') == zone_id]
    if len(zones) != 1:
        raise ValueError(f'destination zone {zone_id!r} missing or ambiguous')
    zone = zones[0]
    target_id = zone.get('target_ref')
    targets = [a for a in environment.get('assets', []) if a.get('id') == target_id]
    if not target_id or len(targets) != 1:
        raise ValueError(f'destination {zone_id}: physical target {target_id!r} missing or ambiguous')
    target = targets[0]
    if target.get('frame') != 'world' or zone.get('frame') != 'world':
        raise ValueError('physical destination requires resolved world target and zone frames')
    if (target.get('collision') or {}).get('enabled') is not True:
        raise ValueError(f'target {target_id}: collision geometry must be enabled')
    usable = target.get('usable_placement')
    local = zone.get('placement_local')
    if not isinstance(usable, dict):
        raise ValueError(f'target {target_id}: missing usable_placement metadata')
    if not isinstance(local, dict):
        raise ValueError(f'destination {zone_id}: missing placement_local metadata')
    lp, lr = pose(local)
    dims = dimensions(local)
    contains(usable, lp, lr, dims)
    tp, tr = pose(target)
    world = [a+b for a, b in zip(tp, apply(tr, lp))]
    orient = multiply(tr, lr)
    if check_projection:
        zp, zr = pose(zone)
        if (math.dist(world, zp) > 1e-7 or
                max(abs(a-b) for row, other in zip(orient, zr) for a, b in zip(row, other)) > 1e-7 or
                any(abs(a-b) > 1e-7 for a, b in zip(dims, dimensions(zone)))):
            raise ValueError(f'destination {zone_id}: stale world-space destination; regenerate from target-local placement')
    return {'id': zone_id, 'target_id': target_id, 'frame_id': 'world',
            'pose_xyz': world, 'pose_rpy': angles(orient), 'dimensions': dims,
            'placement_local': local, 'usable_placement': usable,
            'target_pose_xyz': tp, 'target_pose_rpy': angles(tr),
            'physical_destination_contract': 'target_local/v1'}


def check_object_containment(destination, object_pose, object_dimensions, clearance=0.):
    values = vector(object_pose, 'object pose', 7)
    x, y, z, w = values[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError('invalid object quaternion')
    x, y, z, w = [v/norm for v in (x, y, z, w)]
    rot = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
           [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
           [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    contains(destination, values[:3], rot, vector(object_dimensions, 'object dimensions'), clearance)


def is_destination_zone(zone):
    return (zone.get('target_ref') or zone.get('placement_local') or
            zone.get('type') == 'place_zone' or zone.get('role') in ('place', 'place_zone'))


def destination_ids(environment, task):
    """Include task references, so deleting the referenced zone cannot pass vacuously."""
    ids = {z.get('id') for z in environment.get('task_zones', []) if is_destination_zone(z)}
    place_ref = (task.get('place') or {}).get('target_ref')
    if place_ref:
        ids.add(place_ref)
    for rule in task.get('rules', []):
        if rule.get('destination'):
            ids.add(rule['destination'])
    return sorted(ids, key=str)
