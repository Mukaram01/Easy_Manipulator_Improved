"""Strict execution profile of detected_objects/v1; no ROS or authored-scene writes."""
import copy
import math
from urllib.parse import quote


def vector(value, size, name):
    if (not isinstance(value, (list, tuple)) or len(value) != size or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in value)):
        raise ValueError(f'{name} must contain {size} finite numbers')
    return list(value)


def zone(cell, zone_id):
    matches = [z for z in cell['environment']['task_zones'] if z['id'] == zone_id]
    if len(matches) != 1:
        raise ValueError(f'zone is missing or ambiguous: {zone_id}')
    result = matches[0]
    if result.get('frame', 'world') != 'world':
        raise ValueError(f'zone {zone_id} requires a world transform')
    vector(result['pose_xyz'], 3, 'zone pose')
    if any(v <= 0 for v in vector(result['dimensions'], 3, 'zone dimensions')):
        raise ValueError('zone dimensions must be positive')
    vector(result.get('pose_rpy', [0, 0, 0]), 3, 'zone orientation')
    return result


def task_request(raw, cell):
    result = copy.deepcopy(raw)
    if result.get('action') != 'pick_and_place':
        raise ValueError('action must be pick_and_place')
    for key in ('target_class', 'source_zone', 'destination_zone'):
        if not isinstance(result.get(key), str) or not result[key].strip():
            raise ValueError(f'missing task {key}')
    for key, default in (('max_age_seconds', 2.0), ('min_confidence', 0.0)):
        result.setdefault(key, default)
        vector([result[key]], 1, key)
    if result['max_age_seconds'] <= 0 or not 0 <= result['min_confidence'] <= 1:
        raise ValueError('invalid freshness/confidence policy')
    zone(cell, result['source_zone'])
    zone(cell, result['destination_zone'])
    return result


def normalize(snapshot, now, geometry):
    """World-only first adapter. Other frames fail closed until a TF adapter exists."""
    if snapshot.get('schema_version') != 'detected_objects/v1':
        raise ValueError('schema_version must be detected_objects/v1')
    if not isinstance(snapshot.get('objects'), list) or not snapshot['objects']:
        raise ValueError('objects must be a nonempty list')
    objects, seen = [], set()
    for raw in snapshot['objects']:
        oid = raw.get('object_id')
        if not isinstance(oid, str) or not oid.strip() or oid in seen:
            raise ValueError('object_id must be nonempty and unique')
        seen.add(oid)
        label = raw.get('class_id')
        if not isinstance(label, str) or not label.strip():
            raise ValueError(f'{oid}: class_id required')
        pose = raw['pose']
        if pose.get('frame_id') != 'world':
            raise ValueError(f'{oid}: no valid world transform for {pose.get("frame_id")}')
        xyz = vector(pose['xyz'], 3, 'pose.xyz')
        rpy = vector(pose['rpy'], 3, 'pose.rpy')
        dims = raw['dimensions']
        if isinstance(dims, dict):
            dims = [dims[a] for a in ('x', 'y', 'z')]
        dims = vector(dims, 3, 'dimensions')
        if any(v <= 0 for v in dims):
            raise ValueError(f'{oid}: dimensions must be positive')
        confidence = vector([raw['confidence']], 1, 'confidence')[0]
        stamp = vector([raw['timestamp']], 1, 'timestamp')[0]
        if not 0 <= confidence <= 1 or stamp > now + 0.05:
            raise ValueError(f'{oid}: invalid confidence or future timestamp')
        objects.append(dict(id='runtime::' + quote(oid, safe=''), object_id=oid,
                            class_id=label, confidence=confidence, timestamp=stamp,
                            frame_id='world', source_frame=pose['frame_id'], shape='BOX',
                            pose=xyz + geometry.quaternion_from_rpy(rpy), dimensions=dims,
                            grasp=copy.deepcopy(raw.get('grasp'))))
    return objects


def contained(obj, region, geometry):
    region_pose = region['pose_xyz'] + geometry.quaternion_from_rpy(region.get('pose_rpy', [0, 0, 0]))
    local = geometry.compose_pose(geometry.inverse_pose(region_pose), obj['pose'])
    extents = geometry.oriented_box_extents(dict(target_pose=local, target_dimensions=obj['dimensions']))
    return all(abs(p) + e / 2 <= d / 2 + 1e-9
               for p, e, d in zip(local[:3], extents, region['dimensions']))


def filter_targets(objects, task, cell, now, geometry):
    eligible, rejected = [], {}
    region = zone(cell, task['source_zone'])
    for obj in objects:
        reason = ('class_mismatch' if obj['class_id'] != task['target_class'] else
                  'stale' if now - obj['timestamp'] > task['max_age_seconds'] else
                  'low_confidence' if obj['confidence'] < task['min_confidence'] else
                  'outside_source_zone' if not contained(obj, region, geometry) else None)
        if reason:
            rejected[obj['id']] = reason
        else:
            eligible.append(obj)
    return sorted(eligible, key=lambda o: (-o['confidence'], o['id'])), rejected


def replay_snapshot(template, now):
    """Stamp a new static replay observation once at acquisition, never renew live data."""
    result = copy.deepcopy(template)
    if result.get('source', {}).get('mode') != 'replayed_snapshot':
        raise ValueError('replay template must declare source.mode=replayed_snapshot')
    for obj in result['objects']:
        obj['timestamp'] = now - obj.pop('age_seconds', 0.0)
    return result


def scene_diff(objects):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    result = PlanningScene(is_diff=True)
    for obj in objects:
        pose = Pose()
        (pose.position.x, pose.position.y, pose.position.z,
         pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = obj['pose']
        item = CollisionObject(id=obj['id'], operation=CollisionObject.ADD)
        item.header.frame_id = 'world'
        item.pose = pose
        item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in obj['dimensions']])]
        item.primitive_poses = [Pose()]
        item.primitive_poses[0].orientation.w = 1.0
        result.world.collision_objects.append(item)
    return result
