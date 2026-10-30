"""One physical destination contract; metres, radians and ROS fixed-axis RPY.

Usable boxes are reviewed free space in the physical asset frame (after mesh
origin/scale calibration), never mesh bounding boxes. World zone poses are
checked projections, not an alternative destination. Motion feasibility is
established separately by the complete collision-aware cycle planner.
"""
import copy
import itertools
import math

# The full-cycle planner accepts a planned placement within this distance of the
# requested place. Allocated places keep this much extra room from neighbours and
# region walls, so an object that lands within tolerance still keeps its clearance
# and passes the planner's containment check.
PLACEMENT_TOLERANCE_M = 0.003


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


def margins(outer, center, orient, dims, clearance=0.):
    """Room (m) left on each axis of `outer` after a box keeps `clearance` from its walls."""
    origin, rot = pose(outer)
    bound = dimensions(outer)
    inv = transpose(rot)
    room = [math.inf]*3
    for signs in itertools.product((-1, 1), repeat=3):
        offset = apply(orient, [s*d/2 for s, d in zip(signs, dims)])
        local = apply(inv, [c+o-b for c, o, b in zip(center, offset, origin)])
        room = [min(r, d/2-abs(v)-clearance) for r, v, d in zip(room, local, bound)]
    return room


def contains(outer, center, orient, dims, clearance=0.):
    if any(r < -1e-8 for r in margins(outer, center, orient, dims, clearance)):
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


class PhysicalDestinationContextError(ValueError):
    """Missing, corrupt or stale physical context; fallback is not permitted."""


class RequestedLocalPoseError(ValueError):
    """Invalid requested placement in an otherwise valid physical context."""


def resolve_local_destination(environment, asset_ref, region_ref, local_pose):
    """Resolve a selected pose without moving the authored containment region.

    Structural checks run first so a bad request cannot mask stale geometry.
    Object extents are checked separately by check_object_containment once known.
    """
    try:
        baseline = resolve_destination(environment, region_ref)
    except (ValueError, TypeError, AttributeError, KeyError) as exc:
        raise PhysicalDestinationContextError(
            f'destination {region_ref}: invalid physical context: {exc}') from exc
    if baseline['target_id'] != asset_ref:
        raise PhysicalDestinationContextError(
            f"destination {region_ref}: target mismatch {baseline['target_id']!r} != {asset_ref!r}")
    try:
        if not isinstance(local_pose, dict):
            raise ValueError('requested local pose requires a mapping')
        xyz = vector(local_pose.get('xyz_m'), 'requested local xyz')
        rpy = vector(local_pose.get('rpy_rad'), 'requested local rpy')
        orient = rotation(rpy)
        contains(baseline['placement_local'], xyz, orient, [0., 0., 0.])
        contains(baseline['usable_placement'], xyz, orient, [0., 0., 0.])
    except ValueError as exc:
        raise RequestedLocalPoseError(f'destination {region_ref}: {exc}') from exc
    result = copy.deepcopy(baseline)
    result['region_local'] = result['placement_local']
    result['placement_local'] = {'pose_xyz': xyz, 'pose_rpy': rpy,
                                 'dimensions': list(baseline['dimensions'])}
    tr = rotation(baseline['target_pose_rpy'])
    result['pose_xyz'] = [a+b for a, b in zip(baseline['target_pose_xyz'], apply(tr, xyz))]
    result['pose_rpy'] = angles(multiply(tr, orient))
    return result


def containment_margins(destination, object_pose, object_dimensions, clearance=0.):
    """Per-axis room of every box check_object_containment enforces, in the same frames."""
    values = vector(object_pose, 'object pose', 7)
    x, y, z, w = values[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-9:
        raise ValueError('invalid object quaternion')
    x, y, z, w = [v/norm for v in (x, y, z, w)]
    rot = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
           [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
           [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    dims = vector(object_dimensions, 'object dimensions')
    if 'region_local' in destination:
        inv = transpose(rotation(destination['target_pose_rpy']))
        center = apply(inv, [a-b for a, b in zip(values[:3], destination['target_pose_xyz'])])
        orient = multiply(inv, rot)
        return {'placement region': margins(destination['region_local'], center, orient, dims, clearance),
                'usable placement': margins(destination['usable_placement'], center, orient, dims, clearance)}
    return {'placement region': margins(destination, values[:3], rot, dims, clearance)}


def check_object_containment(destination, object_pose, object_dimensions, clearance=0.):
    if any(r < -1e-8 for room in containment_margins(destination, object_pose, object_dimensions,
                                                     clearance).values() for r in room):
        raise ValueError('placement/object outside target usable placement geometry')


def placement_fit_problems(destination, object_dimensions, clearance, floor_z=None,
                           tolerance=PLACEMENT_TOLERANCE_M):
    """Why an object cannot be placed at this destination, before any motion is planned.

    The object is put where the full-cycle planner targets it (seated on a bound floor, in the
    destination orientation). The planner accepts a placement up to `tolerance` from that target
    in any direction and then enforces check_object_containment on the achieved pose; on each
    region axis that deviation can fall entirely along the axis, so each axis needs `tolerance`
    of room. Returns readable problems, empty when the placement fits.
    """
    from full_cycle_preplanner import seated_destination
    from perceived_object_grasp_plan import quaternion_from_rpy
    orientation = quaternion_from_rpy(destination['pose_rpy'])
    dims = vector(object_dimensions, 'object dimensions')
    target, _ = seated_destination(destination['pose_xyz'], orientation, dims, floor_z)
    problems = []
    for box, room in containment_margins(destination, target + list(orientation), dims, clearance).items():
        for axis, r in zip('xyz', room):
            if r < tolerance-1e-8:
                limit = clearance+r-tolerance
                fix = (f'reduce the placement clearance to at most {limit:.4f} m, or choose another place'
                       if limit >= -1e-12 else 'no clearance fits: choose another place or a larger region')
                problems.append(
                    f'{axis} of the {box}: the {dims[0]:g} x {dims[1]:g} x {dims[2]:g} m object with '
                    f'{clearance:g} m clearance leaves {round(r*1000, 1)+0.:.1f} mm, but planning needs '
                    f'{tolerance*1000:g} mm for its placement tolerance; {fix}')
    return problems


def _region_frame(destination):
    """The containment region check_object_containment enforces, as (origin, rotation, half extents)."""
    region = destination.get('region_local', destination['placement_local'])
    rp, rr = pose(region)
    tr = rotation(destination['target_pose_rpy'])
    origin = [a+b for a, b in zip(destination['target_pose_xyz'], apply(tr, rp))]
    return rp, rr, multiply(tr, rr), origin, [d/2 for d in dimensions(region)]


def _quaternion_matrix(values):
    qx, qy, qz, qw = values
    return [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]


def _footprint(to_region, orient, dims):
    local = multiply(to_region, orient)
    return [sum(abs(v)*d/2 for v, d in zip(row, dims)) for row in local]


def _occupied_boxes(destination, occupied):
    _, _, world_rot, origin, _ = _region_frame(destination)
    to_region = transpose(world_rot)
    boxes = []
    for world_pose, dims in occupied:
        values = vector(world_pose, 'occupied pose', 7)
        boxes.append((apply(to_region, [a-b for a, b in zip(values[:3], origin)]),
                      _footprint(to_region, _quaternion_matrix(values[3:]), vector(dims, 'occupied dimensions'))))
    return boxes


def _overlapping(center, half, boxes, clearance):
    # Footprints only (region x/y): a candidate stands at region-centre height while a placed part rests on the floor,
    # so comparing heights would let a slot land on top of a resting part (G4 plan-only, 2026-10-30). Conservative:
    # anything over a footprint occupies it.
    return [i for i, (c, h) in enumerate(boxes)
            if all(abs(a-b) < x+y+clearance-1e-9 for a, b, x, y in zip(center[:2], c[:2], half[:2], h[:2]))]


def local_to_world_pose(destination, local_pose):
    """A target-local {xyz_m, rpy_rad} placement as a world [x, y, z, qx, qy, qz, qw] pose."""
    tr = rotation(destination['target_pose_rpy'])
    xyz = [a+b for a, b in zip(destination['target_pose_xyz'], apply(tr, vector(local_pose['xyz_m'], 'local xyz')))]
    m = multiply(tr, rotation(local_pose['rpy_rad']))
    w = math.sqrt(max(0., 1+m[0][0]+m[1][1]+m[2][2]))/2
    x = math.copysign(math.sqrt(max(0., 1+m[0][0]-m[1][1]-m[2][2]))/2, m[2][1]-m[1][2])
    y = math.copysign(math.sqrt(max(0., 1-m[0][0]+m[1][1]-m[2][2]))/2, m[0][2]-m[2][0])
    z = math.copysign(math.sqrt(max(0., 1-m[0][0]-m[1][1]+m[2][2]))/2, m[1][0]-m[0][1])
    return xyz + [x, y, z, w]


def place_conflicts(destination, local_pose, object_dimensions, clearance=0., occupied=(), separation=0.):
    """Indices of `occupied` (world 7-pose, dimensions) whose footprint a chosen place overlaps.

    Footprints must stay `clearance` + `separation` apart.
    """
    rp, rr, _, _, _ = _region_frame(destination)
    to_region = transpose(rr)
    center = apply(to_region, [a-b for a, b in zip(vector(local_pose['xyz_m'], 'local xyz'), rp)])
    half = _footprint(to_region, rotation(local_pose['rpy_rad']), vector(object_dimensions, 'object dimensions'))
    return _overlapping(center, half, _occupied_boxes(destination, occupied), clearance+separation)


def wall_margin_ok(destination, local_pose, object_dimensions, clearance=0., margin=0.):
    """Does a place keep clearance + margin to the region walls horizontally? (Height: containment.)"""
    rp, rr, _, _, halves = _region_frame(destination)
    to_region = transpose(rr)
    center = apply(to_region, [a-b for a, b in zip(vector(local_pose['xyz_m'], 'local xyz'), rp)])
    half = _footprint(to_region, rotation(local_pose['rpy_rad']), vector(object_dimensions, 'object dimensions'))
    return all(abs(c)+h+clearance+margin <= w+1e-8 for c, h, w in zip(center[:2], half[:2], halves[:2]))


def placement_slots(destination, object_dimensions, clearance=0., occupied=(), step=0.01, separation=0.):
    """Free target-local poses for one more object inside the authored region.

    Candidates lie on a grid inside the containment region (the one
    check_object_containment enforces), packed from the region's -x/-y corner
    so later objects keep room. `occupied` holds (world 7-pose, dimensions) of
    objects already there; a slot whose footprint, inflated by `clearance`,
    overlaps one is dropped. Gripper clearance is not modelled here: the
    full-cycle planner remains the only feasibility check for every slot.
    Objects keep `clearance` + `separation` from each other and from the region walls.
    """
    rp, rr, _, _, (hx, hy, hz) = _region_frame(destination)
    ox, oy, oz = (d/2 for d in vector(object_dimensions, 'object dimensions'))
    boxes = _occupied_boxes(destination, occupied)

    def axis(h, o, i):
        lo, hi = -h+o+clearance+separation, h-o-clearance-separation
        if lo > hi+1e-9:
            return []
        # The authored region is centred on its frame: keep the centre-line on the grid.
        inner = [k*step for k in range(int(math.ceil(lo/step-1e-9)), int(math.floor(hi/step+1e-9))+1)]
        # Flush against each occupied footprint, so a placed object's planning tolerance
        # does not cost a whole grid step of room.
        flush = [c[i]+side*(half[i]+o+clearance+separation) for c, half in boxes for side in (-1, 1)]
        return sorted({round(v, 9) for v in [lo, hi] + inner + flush if lo-1e-9 <= v <= hi+1e-9})

    if oz+clearance > hz+1e-8:
        return []
    slots = []
    for x in axis(hx, ox, 0):
        for y in sorted(axis(hy, oy, 1), key=abs):
            if _overlapping([x, y, 0.], [ox, oy, oz], boxes, clearance+separation):
                continue
            slots.append({'xyz_m': [a+b for a, b in zip(rp, apply(rr, [x, y, 0.]))],
                          'rpy_rad': angles(rr)})
    return slots


def place_options(destination, object_dimensions, clearance=0., robot_xyz=None, limit=12):
    """Fixed places an operator can choose for one object in the empty region, with a description.

    Places form an evenly spread grid of as many places as fit with clearance +
    PLACEMENT_TOLERANCE_M to the walls and between neighbours. They never move
    when other objects are chosen, so a saved choice keeps meaning the same spot.
    Descriptions say how far along the region's long axis a place lies from the robot
    (`robot_xyz`, world); returns [(target-local pose, description)].
    """
    rp, rr, world_rot, origin, halves = _region_frame(destination)
    dims = vector(object_dimensions, 'object dimensions')
    if dims[2]/2+clearance > halves[2]+1e-8:
        return []
    pitch = [d+clearance+PLACEMENT_TOLERANCE_M for d in dims[:2]]
    axes = []
    for h, d, p in zip(halves[:2], dims[:2], pitch):
        lo, hi = -h+d/2+clearance+PLACEMENT_TOLERANCE_M, h-d/2-clearance-PLACEMENT_TOLERANCE_M
        if lo > hi+1e-9:
            return []
        count = 1 + int(math.floor((hi-lo)/p+1e-9))
        axes.append([0.] if count == 1 else [lo+(hi-lo)*k/(count-1) for k in range(count)])
    long_axis = 0 if halves[0] >= halves[1] else 1
    toward = [0., 0.]
    if robot_xyz is not None:  # Order and describe from the end nearest the robot.
        robot = apply(transpose(world_rot), [a-b for a, b in zip(vector(robot_xyz, 'robot xyz'), origin)])
        toward = [-1. if robot[i] < 0 else 1. for i in range(2)]
    along = sorted(axes[long_axis], key=lambda v: toward[long_axis]*v, reverse=True)
    across = sorted(axes[1-long_axis], key=lambda v: toward[1-long_axis]*v, reverse=True)
    words = {1: ['centre'], 2: ['near end', 'far end'], 3: ['near end', 'middle', 'far end']}
    places = []
    for i, a in enumerate(along):
        where = (words.get(len(along)) or [f'position {k} from the near end' for k in range(1, len(along)+1)])[i]
        for j, c in enumerate(across):
            xy = [0., 0.]
            xy[long_axis], xy[1-long_axis] = a, c
            pose = {'xyz_m': [u+v for u, v in zip(rp, apply(rr, [xy[0], xy[1], 0.]))], 'rpy_rad': angles(rr)}
            places.append((pose, where if len(across) == 1 else f'{where}, row {j+1} of {len(across)}'))
    return places[:limit]


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
