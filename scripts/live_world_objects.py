#!/usr/bin/env python3
"""Calibrated live EPD observations -> world objects for the cell (perception only; never robot motion).

``world_snapshot`` turns one normalized live EPD snapshot (workcell_perception_snapshot/v1, camera optical
frame) into a ``detected_objects/v1`` world snapshot using the cell's qualified camera-to-world calibration.
Every object keeps its camera-frame measurement, class, confidence, sensor timestamp and EPD object id.

Refused (never transformed into planning input): a missing/unqualified calibration, a snapshot or object in
another frame or from another camera, a stale or future-stamped observation, a missing 3D measurement,
insufficient valid depth, and a world position outside the cell's workspace bounds.

A real object is *not* present in Gazebo/MoveIt by itself. It becomes a planning object only when the cell
authors a virtual counterpart for it (``perception.live_counterparts``: label, box size) and the detection is
geometrically consistent with that counterpart. EPD's label is reported, never verified: the counterpart
binding records ``class_verified: false``; a mislabelled detection whose measured extent does not fit the
counterpart is refused.

``publish`` runs this continuously against EPD's live contract topic and publishes the world snapshot (JSON),
RViz markers, a TF for the calibrated optical frame and, optionally, the counterparts as MoveIt collision
objects (planning scene only).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

PRODUCER = 'epd_live'
PROVENANCE = 'REAL_CAMERA_LIVE'
LIVE_CEILING_S = 2.0               # task_intent_v2 OBSERVATION_PRODUCER_CEILING_S['live_sensor']
MIN_VALID_DEPTH_RATIO = 0.5
COUNTERPART_TOLERANCE_M = 0.02     # default slack between EPD's visible extent and the authored box
SNAPSHOT_SCHEMA = 'workcell_perception_snapshot/v1'


def _finite(values, n):
    try:
        out = [float(v) for v in values]
    except (TypeError, ValueError):
        return None
    return out if len(out) == n and all(math.isfinite(v) for v in out) else None


def _quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _matrix_quaternion(m):
    from camera_world_calibration import rotation_to_quaternion
    return rotation_to_quaternion(m)


def workspace_bounds(environment):
    bounds = (environment.get('workspace') or {}).get('bounds') or {}
    keys = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')
    if not all(isinstance(bounds.get(k), (int, float)) for k in keys):
        return None
    return [(bounds['x_min'], bounds['x_max']), (bounds['y_min'], bounds['y_max']), (bounds['z_min'], bounds['z_max'])]


def counterparts(environment):
    """The cell's authored virtual counterparts for live objects (explicit; never inferred)."""
    rows = ((environment.get('perception') or {}).get('live_counterparts')) or []
    out = []
    for row in rows:
        size = _finite(row.get('size_m') or [], 3)
        if not row.get('id') or not row.get('label') or row.get('shape', 'box') != 'box' or not size or min(size) <= 0:
            raise ValueError(f'COUNTERPART_INVALID: {row!r} needs id, label, shape box and a positive size_m')
        out.append(dict(id=row['id'], label=row['label'], shape='box', size_m=size,
                        tolerance_m=float(row.get('extent_tolerance_m', COUNTERPART_TOLERANCE_M))))
    return out


def _fits(extent, counterpart):
    """EPD's visible extent fits inside the authored box (any orientation) and covers its largest side."""
    seen, box, tol = sorted(extent), sorted(counterpart['size_m']), counterpart['tolerance_m']
    return all(s <= b + tol for s, b in zip(seen, box)) and seen[-1] >= box[-1] - tol


def world_snapshot(snapshot, calibration, binding, environment, *, now_ns, age_s=None):
    """detected_objects/v1 world snapshot plus refusals. ``calibration``: a loaded qualified record or None."""
    refused, objects, blockers = [], [], []
    camera_frame = binding['optical_frame']
    if calibration is None:
        blockers.append('CALIBRATION_MISSING: no qualified camera-to-world calibration')
    elif calibration.get('source_frame') != camera_frame:
        blockers.append(f"CALIBRATION_MISMATCH: calibrated frame {calibration.get('source_frame')!r}")
    if snapshot.get('schema_version') != SNAPSHOT_SCHEMA or snapshot.get('runtime_mode') != 'live':
        blockers.append('SNAPSHOT_INVALID: not a live workcell_perception_snapshot/v1')
    if snapshot.get('camera_id') != binding['camera_id']:
        blockers.append(f"CAMERA_MISMATCH: snapshot camera {snapshot.get('camera_id')!r} is not {binding['camera_id']!r}")
    if snapshot.get('frame_id') != camera_frame:
        blockers.append(f"FRAME_MISMATCH: snapshot frame {snapshot.get('frame_id')!r} is not {camera_frame!r}")
    stamp = snapshot.get('timestamp_ns')
    if not isinstance(stamp, int):
        blockers.append('TIMESTAMP_MISSING: the snapshot has no sensor timestamp')
        age = math.inf
    else:
        age = age_s if age_s is not None else (now_ns - stamp) / 1e9
    if age > LIVE_CEILING_S:
        blockers.append(f'STALE: observation {age:.2f} s old (live limit {LIVE_CEILING_S:.1f} s)')
    elif age < -0.5:
        blockers.append(f'CLOCK_DOMAIN: observation stamped {-age:.2f} s in the future')
    bounds = workspace_bounds(environment)
    if bounds is None:
        blockers.append('WORKSPACE_UNDEFINED: the cell has no workspace bounds')
    authored = counterparts(environment)
    transform = np.asarray(calibration['camera_to_world']['matrix'], float) if not blockers else None

    for index, item in enumerate(snapshot.get('objects') or []):
        object_id = str(item.get('object_id') or item.get('track_id') or f'epd_{index}')
        pose = item.get('pose') or {}
        attributes = item.get('attributes') or {}
        position = _finite(pose.get('position') or [], 3)
        orientation = _finite(pose.get('orientation_xyzw') or [], 4)
        extent = _finite(item.get('dimensions_xyz') or [], 3)
        ratio = attributes.get('valid_depth_ratio')
        reasons = list(blockers)
        if pose.get('frame_id') != camera_frame:
            reasons.append(f"FRAME_MISMATCH: object frame {pose.get('frame_id')!r}")
        if position is None:
            reasons.append('NO_3D_MEASUREMENT: no finite camera-frame position')
        if extent is None or min(extent) <= 0:
            reasons.append('NO_EXTENT: no measured dimensions')
        if not isinstance(ratio, (int, float)) or ratio < MIN_VALID_DEPTH_RATIO:
            reasons.append(f'INVALID_DEPTH: valid depth ratio {ratio!r} below {MIN_VALID_DEPTH_RATIO}')
        camera = dict(frame_id=camera_frame, position=position, orientation_xyzw=orientation,
                      visible_extent_m=extent, roi_xywh=attributes.get('roi_xywh'), valid_depth_ratio=ratio)
        world = None
        if not reasons:
            world = (transform[:3, :3] @ np.array(position) + transform[:3, 3]).tolist()
            if not all(lo <= v <= hi for v, (lo, hi) in zip(world, bounds)):
                reasons.append(f'OUT_OF_WORKSPACE: world position {[round(v, 3) for v in world]} is outside the cell bounds')
        base = dict(object_id=object_id.replace(':', '_'), epd_object_id=object_id, class_id=item.get('label'),
                    class_verified=False, confidence=item.get('confidence'), age_seconds=age,
                    perception=dict(producer=PRODUCER, source_stamp_ns=stamp, camera=camera))
        if reasons:
            refused.append(dict(base, world_xyz=world, reasons=reasons))
            continue
        rotation = transform[:3, :3] @ (_quaternion_matrix(orientation) if orientation else np.eye(3))
        world_pose = dict(frame_id='world', xyz=world, orientation_xyzw=_matrix_quaternion(rotation))
        objects.append(dict(base, pose=world_pose, dimensions=[max(v, 1e-3) for v in extent], planning=None))

    # Explicit virtual counterparts: one consistent detection per counterpart, else nothing for that counterpart.
    candidates = {c['id']: [o for o in objects if o['class_id'] == c['label'] and _fits(o['dimensions'], c)] for c in authored}
    for obj in objects:
        matches = [c for c in authored if any(o is obj for o in candidates[c['id']])]
        same_label = [c for c in authored if c['label'] == obj['class_id']]
        if not same_label:
            obj['planning'] = dict(status='BLOCKED', reason=f"no virtual counterpart is authored for label {obj['class_id']!r}")
        elif not matches:
            obj['planning'] = dict(status='BLOCKED', reason=f"measured extent {[round(v, 3) for v in obj['dimensions']]} m does "
                                   f"not fit the authored {obj['class_id']!r} counterpart (label unverified)")
        elif len(matches) > 1 or len(candidates[matches[0]['id']]) > 1:
            obj['planning'] = dict(status='BLOCKED', reason='ambiguous: more than one detection or counterpart matches')
        else:
            c = matches[0]
            obj['planning'] = dict(status='READY', counterpart_id=c['id'], shape='box', size_m=c['size_m'],
                                   pose=obj['pose'], class_identity='epd_label_unverified_geometry_consistent')
    return dict(schema_version='detected_objects/v1',
                source=dict(mode='live_epd', producer=PRODUCER, provenance=PROVENANCE, camera_source='realsense_d435i',
                            camera_id=binding['camera_id'], clock_domain='sensor_time', snapshot_timestamp_ns=stamp,
                            age_seconds=age if math.isfinite(age) else None,
                            calibration=None if calibration is None else dict(
                                record=binding.get('calibration', {}).get('record'),
                                record_sha256=binding.get('calibration', {}).get('record_sha256'),
                                camera_serial=calibration.get('camera_serial'),
                                rms_m=calibration['residuals']['rms_m'],
                                max_leave_one_out_m=calibration['residuals']['max_leave_one_out_m']),
                            blockers=blockers,
                            geometry='planning geometry comes only from an authored virtual counterpart'),
                objects=objects, refused_objects=refused)


# --- ROS publisher (RViz markers, TF, optional planning scene) ---------------------------------------------
def _markers(world, stamp):
    from visualization_msgs.msg import Marker, MarkerArray
    array = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)

    def marker(ns, mid, kind, xyz, scale, rgba, text='', q=(0, 0, 0, 1)):
        m = Marker(); m.header.frame_id = 'world'; m.header.stamp = stamp; m.ns = ns; m.id = mid; m.type = kind
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        return m
    for i, obj in enumerate(world['objects']):
        xyz = obj['pose']['xyz']
        ready = obj['planning']['status'] == 'READY'
        array.markers.append(marker('epd_world_centroid', i, Marker.SPHERE, xyz, (0.02, 0.02, 0.02),
                                    (0.1, 0.8, 0.1, 1.0) if ready else (0.9, 0.6, 0.1, 1.0)))
        label = f"{obj['class_id']} {obj['confidence']:.2f} (EPD label, unverified)\n" + \
            (f"counterpart {obj['planning']['counterpart_id']}" if ready else obj['planning']['reason'])
        array.markers.append(marker('epd_world_label', i, Marker.TEXT_VIEW_FACING,
                                    [xyz[0], xyz[1], xyz[2] + 0.08], (0, 0, 0.025), (1, 1, 1, 1), label))
        if ready:
            array.markers.append(marker('virtual_counterpart', i, Marker.CUBE, xyz, obj['planning']['size_m'],
                                        (0.2, 0.5, 1.0, 0.45), q=obj['pose']['orientation_xyzw']))
    for i, obj in enumerate(world['refused_objects']):
        if obj.get('world_xyz'):
            array.markers.append(marker('epd_refused', i, Marker.SPHERE, obj['world_xyz'], (0.02, 0.02, 0.02),
                                        (0.8, 0.1, 0.1, 1.0)))
    return array


def publish(args):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from visualization_msgs.msg import MarkerArray
    import camera_world_calibration as cwc
    import live_perception_session as lps
    binding = lps.camera_binding(args.cell)
    calibration = cwc.load_qualified(args.cell, binding)
    environment = yaml.safe_load((Path(args.cell) / 'environment.yaml').read_text()) or {}
    counterparts(environment)                                    # authored counterparts must be valid up front
    rclpy.init()
    node = rclpy.create_node('workcell_live_world_objects')
    serial = cwc._camera_serial(node, binding['rgb_topic'])
    if serial != calibration['camera_serial']:
        raise cwc.CalibrationError(f"CAMERA_IDENTITY_MISMATCH: running camera serial {serial!r} is not the calibrated "
                                   f"{calibration['camera_serial']!r}")
    tf = TransformStamped()
    tf.header.frame_id = 'world'
    tf.child_frame_id = args.calibrated_frame
    m = calibration['camera_to_world']
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = m['translation_xyz']
    (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
     tf.transform.rotation.w) = m['quaternion_xyzw']
    broadcaster = StaticTransformBroadcaster(node)
    tf.header.stamp = node.get_clock().now().to_msg()
    broadcaster.sendTransform(tf)
    out_json = node.create_publisher(String, '/workcell_studio/live_world_objects_json', 10)
    out_markers = node.create_publisher(MarkerArray, '/workcell_studio/live_world_objects/markers', 10)
    planning = None
    if args.apply_planning_scene:
        planning = _PlanningScene(node, args.planning_frame)
    latest = {}

    def on_snapshot(msg):
        try:
            snap = json.loads(msg.data)
        except ValueError:
            return
        world = world_snapshot(snap, calibration, binding, environment, now_ns=time.time_ns())
        latest['world'] = world
        out_json.publish(String(data=json.dumps(world)))
        out_markers.publish(_markers(world, node.get_clock().now().to_msg()))
        if planning:
            planning.update(world)
        if args.output:
            Path(args.output).write_text(yaml.safe_dump(world, sort_keys=False))

    node.create_subscription(String, '/workcell_studio/epd_detection_snapshot_json', on_snapshot, 10)
    deadline = time.time() + args.duration if args.duration > 0 else math.inf
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if planning:
            planning.clear()
        node.destroy_node(); rclpy.shutdown()
    return 0


class _PlanningScene:
    """Counterparts as MoveIt collision objects (planning scene only; never motion)."""

    def __init__(self, node, frame):
        from moveit_msgs.msg import PlanningScene
        self.node, self.frame, self.applied = node, frame, set()
        self.pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

    def _scene(self, objects):
        from moveit_msgs.msg import PlanningScene
        scene = PlanningScene(); scene.is_diff = True
        scene.world.collision_objects = objects
        return scene

    def update(self, world):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        objects, current = [], set()
        for obj in world['objects']:
            plan = obj['planning']
            if plan['status'] != 'READY':
                continue
            co = CollisionObject(); co.header.frame_id = 'world'; co.id = f"live_{plan['counterpart_id']}"
            box = SolidPrimitive(); box.type = SolidPrimitive.BOX; box.dimensions = list(plan['size_m'])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = plan['pose']['xyz']
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = plan['pose']['orientation_xyzw']
            co.primitives, co.primitive_poses, co.operation = [box], [pose], CollisionObject.ADD
            objects.append(co); current.add(co.id)
        for gone in self.applied - current:
            co = CollisionObject(); co.id = gone; co.header.frame_id = 'world'; co.operation = CollisionObject.REMOVE
            objects.append(co)
        self.applied = current
        if objects:
            self.pub.publish(self._scene(objects))

    def clear(self):
        self.update(dict(objects=[]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('publish', help='live world objects to RViz/TF (and optionally the MoveIt planning scene)')
    p.add_argument('--cell', required=True)
    p.add_argument('--duration', type=float, default=0.0)
    p.add_argument('--output', help='also write the latest detected_objects/v1 world snapshot here')
    p.add_argument('--calibrated-frame', default='calibrated_camera_color_optical_frame')
    p.add_argument('--apply-planning-scene', action='store_true')
    p.add_argument('--planning-frame', default='world')
    t = sub.add_parser('transform', help='one saved live EPD snapshot (JSON) to a world snapshot')
    t.add_argument('--cell', required=True); t.add_argument('snapshot')
    t.add_argument('--age-s', type=float, help='observation age when the snapshot was captured')
    args = parser.parse_args(argv)
    import camera_world_calibration as cwc
    import live_perception_session as lps
    try:
        if args.command == 'publish':
            return publish(args)
        binding = lps.camera_binding(args.cell)
        environment = yaml.safe_load((Path(args.cell) / 'environment.yaml').read_text()) or {}
        snapshot = json.loads(Path(args.snapshot).read_text())
        world = world_snapshot(snapshot, cwc.load_qualified(args.cell, binding), binding, environment,
                               now_ns=time.time_ns(), age_s=args.age_s)
        print(json.dumps(world))
        return 0 if world['objects'] else 3
    except (cwc.CalibrationError, lps.PerceptionUnavailable, OSError, KeyError, ValueError) as exc:
        print(json.dumps(dict(status='BLOCKED', reason=str(exc))))
        return 2


if __name__ == '__main__':
    sys.exit(main())
