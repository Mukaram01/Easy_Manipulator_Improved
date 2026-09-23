"""Bounded BOX extraction intents and geometric prediction, never contact authority.

The existing MoveIt planner still checks the complete robot/private scene. These
helpers only reject poor target/neighbor geometry using its existing FCL ABI;
they never create an ACM allowance or a physical pile certificate.
"""
import ctypes
import math
from functools import lru_cache
from pathlib import Path

TOLERANCE_M = .0001
CORRIDOR_XY_M = .0025
CORRIDOR_HEIGHT_M = .01
CORRIDOR_ANGLE_RAD = .01
MAX_OFFSET_M = .002
MAX_SAMPLES = 20000


class ExtractionFailure(RuntimeError):
    def __init__(self, reason_code, **details):
        self.reason_code = 'NO_VALID_EXTRACTION'
        self.details = dict(failure_kind='extraction', extraction_reason_code=reason_code, **details)
        super().__init__('NO_VALID_EXTRACTION: ' + reason_code + ' ' + repr(details))


def _fail(code, **details):
    raise ExtractionFailure(code, **details)


def _finite(values, count):
    return (isinstance(values, (list, tuple)) and len(values) == count and
            all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in values))


def _pose(values):
    if not _finite(values, 7) or abs(sum(x*x for x in values[3:])-1.) > 1e-6:
        _fail('EXTRACTION_GEOMETRY_INVALID', field='pose')
    return list(values)


def _box(value):
    if (not isinstance(value, dict) or not isinstance(value.get('id'), str) or not value['id'] or
            value.get('shape') != 'BOX' or value.get('frame_id', 'world') != 'world' or
            not _finite(value.get('dimensions'), 3) or any(x <= 0 for x in value['dimensions'])):
        _fail('EXTRACTION_GEOMETRY_INVALID', field='BOX')
    return dict(id=value['id'], dimensions=list(value['dimensions']), pose=_pose(value.get('pose')))


def _inputs(target, neighbors):
    target = _box(target)
    if not isinstance(neighbors, (list, tuple)):
        _fail('EXTRACTION_GEOMETRY_INVALID', field='neighbors')
    boxes = [_box(n) for n in neighbors]
    boxes = [n for n in boxes if n['id'] != target['id']]
    if len({n['id'] for n in boxes}) != len(boxes):
        _fail('EXTRACTION_GEOMETRY_INVALID', field='duplicate neighbor identity')
    return target, sorted(boxes, key=lambda n: n['id'])


@lru_cache(maxsize=1)
def _predicate():
    from ament_index_python.packages import get_package_prefix
    library = ctypes.CDLL(str(Path(get_package_prefix('workcell_builder')) /
                              'lib/libworkcell_support_contact.so'))
    fn = library.workcell_measured_pile_contact
    pointer = ctypes.POINTER(ctypes.c_double)
    fn.argtypes = [pointer]*5 + [ctypes.c_size_t, pointer]
    fn.restype = ctypes.c_bool
    return fn


def _geometry(target, neighbor, pose=None):
    def array(values):
        return (ctypes.c_double*len(values))(*values)
    output = (ctypes.c_double*8)()
    try:
        # n=0 deliberately cannot certify a contact. Only its FCL geometry is used.
        _predicate()(array(target['dimensions']), array(pose or target['pose']),
                     array(neighbor['dimensions']), array(neighbor['pose']), array([]), 0, output)
    except Exception as exc:
        _fail('EXTRACTION_GEOMETRY_UNAVAILABLE', error=str(exc))
    if not all(math.isfinite(x) for x in output) or output[0] < 0 or output[1] < 0:
        _fail('EXTRACTION_GEOMETRY_INVALID', neighbor=neighbor['id'])
    return dict(depth_m=output[0], gap_m=output[1], normal=list(output[2:5]))


def _initial(target, neighbors):
    evidence = {}
    for neighbor in neighbors:
        g = _geometry(target, neighbor)
        if g['depth_m'] > TOLERANCE_M:
            _fail('EXTRACTION_INITIAL_DEPTH', neighbor=neighbor['id'], **g)
        evidence[neighbor['id']] = g
    return evidence


def extraction_intents(target, neighbors, candidate_id, lift_distance):
    """Vertical, aggregate away direction, and at most two individual normals.

    Only current initial contacts/near gaps supply directions. Positive-gap
    neighbors influence intent but never become permitted initial contacts.
    Stable IDs only break equal-distance ordering; no coordinate axis is chosen.
    """
    target, neighbors = _inputs(target, neighbors)
    if (not isinstance(candidate_id, str) or not candidate_id or
            not isinstance(lift_distance, (int, float)) or isinstance(lift_distance, bool) or
            not math.isfinite(lift_distance) or lift_distance <= 0):
        _fail('EXTRACTION_INTENT_INVALID')
    evidence = _initial(target, neighbors)
    directions = []
    vertical_probe = list(target['pose'])
    vertical_probe[2] += min(lift_distance, CORRIDOR_HEIGHT_M)
    by_id = {n['id']: n for n in neighbors}
    for name, g in sorted(evidence.items(), key=lambda item: (item[1]['gap_m'], item[0])):
        length = math.hypot(*g['normal'][:2])
        # Do not amplify tiny horizontal components of a tilted floor normal.
        # A neighbor already cleared by vertical motion is not an away source.
        if (g['gap_m'] <= TOLERANCE_M and length > 1e-9 and
                _geometry(target, by_id[name], vertical_probe)['gap_m'] <= TOLERANCE_M):
            directions.append([-g['normal'][0]/length, -g['normal'][1]/length])
    variants = [[0., 0.]]
    if directions:
        aggregate = [sum(v[k] for v in directions) for k in (0, 1)]
        for direction in [aggregate, *directions[:2]]:
            norm = math.hypot(*direction)
            if norm <= 1e-9:
                continue
            offset = [MAX_OFFSET_M*v/norm for v in direction]
            if not any(math.dist(offset, old) < 1e-12 for old in variants):
                variants.append(offset)
    return [dict(schema='workcell_extraction_intent/v1', object_id=target['id'],
                 candidate_id=candidate_id, variant_id='vertical' if i == 0 else f'geometry_away_{i}',
                 offset_xyz_m=[*offset, float(lift_distance)]) for i, offset in enumerate(variants)]


def _angle(a, b):
    dot = abs(sum(x*y for x, y in zip(a[3:], b[3:])))
    norms = math.sqrt(sum(x*x for x in a[3:])*sum(x*x for x in b[3:]))
    return 2*math.acos(min(1., dot/norms))


def _interpolate(a, b, fraction):
    qa, qb = a[3:], b[3:]
    dot = sum(x*y for x, y in zip(qa, qb))
    if dot < 0:
        qb = [-x for x in qb]; dot = -dot
    if dot > .9995:
        q = [x + fraction*(y-x) for x, y in zip(qa, qb)]
    else:
        theta = math.acos(max(-1., min(1., dot)))
        q = [(math.sin((1-fraction)*theta)*x + math.sin(fraction*theta)*y)/math.sin(theta)
             for x, y in zip(qa, qb)]
    norm = math.sqrt(sum(x*x for x in q))
    return [x + fraction*(y-x) for x, y in zip(a[:3], b[:3])] + [x/norm for x in q]


def _samples(target, intent, poses):
    if poses is None:
        end = [x+y for x, y in zip(target['pose'][:3], intent['offset_xyz_m'])] + target['pose'][3:]
        poses = [target['pose'], end]
    else:
        poses = list(poses)
    if not poses or len(poses) > MAX_SAMPLES:
        _fail('EXTRACTION_SAMPLE_BOUND')
    previous = _pose(poses[0]); count = 1
    yield previous
    for item in poses[1:]:
        current = _pose(item)
        divisions = max(1, math.ceil(math.dist(previous[:3], current[:3])/TOLERANCE_M),
                        math.ceil(_angle(previous, current)/.001))
        count += divisions
        if count > MAX_SAMPLES:
            _fail('EXTRACTION_SAMPLE_BOUND', samples=count)
        for i in range(1, divisions+1):
            # Keep every supplied pose exactly; interpolation adds geometry samples only.
            yield current if i == divisions else _interpolate(previous, current, i/divisions)
        previous = current


def audit_extraction(target, neighbors, intent, poses=None):
    """Conservative geometric expiry audit; MoveIt and live guards remain required.

    Supplied object poses are densified to <=0.1mm translation / 0.001rad.
    Near-only initial pairs must gain clearance but retain strict no-contact
    status. An expired pair returning within the clearance bound is rejected.
    """
    target, neighbors = _inputs(target, neighbors)
    if not isinstance(intent, dict):
        _fail('EXTRACTION_INTENT_INVALID')
    offset = intent.get('offset_xyz_m')
    if (intent.get('schema') != 'workcell_extraction_intent/v1' or intent.get('object_id') != target['id'] or
            not isinstance(intent.get('candidate_id'), str) or not intent['candidate_id'] or
            not _finite(offset, 3) or offset[2] <= 0 or math.hypot(*offset[:2]) > MAX_OFFSET_M + 1e-15):
        _fail('EXTRACTION_INTENT_INVALID')
    initial = _initial(target, neighbors)
    contacts = {n for n, g in initial.items() if g['gap_m'] == 0.}
    near = {n for n, g in initial.items() if 0. < g['gap_m'] <= TOLERANCE_M}
    required = contacts | near
    expired = {}; last_height = 0.; max_depth = 0.; samples = 0
    for index, pose in enumerate(_samples(target, intent, poses)):
        samples += 1
        height = pose[2] - target['pose'][2]
        xy = math.dist(pose[:2], target['pose'][:2]); rotation = _angle(pose, target['pose'])
        active = required - expired.keys()
        if active and (xy > CORRIDOR_XY_M or height < last_height-1e-9 or
                       height > CORRIDOR_HEIGHT_M or rotation > CORRIDOR_ANGLE_RAD):
            name = sorted(active)[0]
            geometry = _geometry(target, next(n for n in neighbors if n['id'] == name), pose)
            _fail('EXTRACTION_CORRIDOR', neighbor=name, sample=index,
                  height_m=height, previous_height_m=last_height, xy_m=xy, rotation_rad=rotation,
                  **geometry)
        last_height = height
        for neighbor in neighbors:
            name = neighbor['id']; g = _geometry(target, neighbor, pose)
            max_depth = max(max_depth, g['depth_m'])
            details = dict(neighbor=name, sample=index, height_m=height, **g)
            if name in expired and g['gap_m'] <= TOLERANCE_M:
                _fail('EXTRACTION_CLEARANCE_REVERSED', **details)
            if g['gap_m'] == 0. and (name not in contacts or name in expired):
                _fail('EXTRACTION_NEW_CONTACT', **details)
            if g['depth_m'] > TOLERANCE_M:
                _fail('EXTRACTION_DEPTH', **details)
            if name in active and g['gap_m'] > TOLERANCE_M:
                expired[name] = dict(sample=index, height_m=height, gap_m=g['gap_m'])
    if required - expired.keys():
        name = sorted(required - expired.keys())[0]
        _fail('EXTRACTION_NOT_SEPARATED', neighbor=name, height_m=last_height,
              gap_m=_geometry(target, next(n for n in neighbors if n['id'] == name), pose)['gap_m'])
    return dict(success=True, initial_contacts=sorted(contacts), near_neighbors=sorted(near),
                expiry=expired, samples=samples, max_depth_m=max_depth,
                physical_contact_authority=False)
