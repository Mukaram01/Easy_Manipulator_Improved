#!/usr/bin/env python3
"""Validate a frozen EPD RGB-D snapshot; optionally apply verified static extrinsics.

Never publishes to ROS, calls planning or executes motion. Simulation timestamps
are retained; replay consumers must not treat these as current ROS wall time.
"""
import argparse
import copy
import json
import math
from pathlib import Path
from epd_snapshot_adapter import validate_normalized_snapshot
from perceived_object_grasp_plan import quaternion_from_rpy, rotate_vector


def transform_snapshot(snapshot, expected_pose):
    source = snapshot.get('source', {})
    pose = source.get('camera_world_pose')
    if (snapshot.get('frame_id') != 'stage_a_camera_optical_frame' or
            source.get('camera_pose_source') != 'live_scene_info' or
            source.get('camera_is_static') is not True or
            source.get('camera_definition_source') != 'live_generate_world_sdf' or
            not isinstance(pose, list) or len(pose) != 7 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose) or
            abs(sum(v*v for v in pose[3:])-1) > 1e-6):
        raise ValueError('BLOCKED: verified static camera world transform unavailable')
    if len(expected_pose) != 6 or not all(math.isfinite(v) for v in expected_pose):
        raise ValueError('invalid expected camera pose')
    expected_q = quaternion_from_rpy(expected_pose[3:])
    if (math.dist(pose[:3], expected_pose[:3]) > 1e-6 or
            abs(abs(sum(a*b for a,b in zip(expected_q,pose[3:])))-1) > 1e-6):
        raise ValueError('BLOCKED: live camera transform differs from configured pose')
    if any('pose' in obj or 'centroid' not in obj for obj in snapshot.get('objects', [])):
        raise ValueError('RGB-D surface transform requires centroid-only observations')
    errors = validate_normalized_snapshot(snapshot)
    if errors:
        raise ValueError('; '.join(errors))
    out = copy.deepcopy(snapshot)
    for obj in out['objects']:
        # Optical right/down/forward -> Gazebo camera forward/left/up.
        x,y,z = obj['centroid']
        rotated = rotate_vector(pose[3:], [z,-x,-y])
        obj['centroid'] = [t+v for t,v in zip(pose[:3],rotated)]
        attrs = obj.get('attributes', {})
        if 'surface_points_optical' in attrs:
            points = attrs.pop('surface_points_optical')
            if (not isinstance(points, list) or any(not isinstance(p, list) or len(p) != 3 or
                    not all(isinstance(v, (int, float)) and math.isfinite(v) for v in p) for p in points)):
                raise ValueError('BLOCKED: invalid measured surface points')
            attrs['measured_surface_points'] = [
                [t+v for t,v in zip(pose[:3], rotate_vector(pose[3:], [p[2],-p[0],-p[1]]))]
                for p in points]
            attrs['measurement_frame'] = 'world'
    out['frame_id'] = 'world'
    return out


def estimate_cube_geometry(obj, profile):
    """Fit one sufficiently visible cube face; declared dimensions are never measurements.

    Geometry is only for explicit offline replay. Fourfold face-yaw symmetry is
    physically equivalent for a cube. Other shapes and unresolved faces block.
    """
    import numpy as np
    import cv2
    attrs = obj.get('attributes', {})
    def blocked(reason):
        raise ValueError('BLOCKED: ' + reason)
    dims = profile.get('dimensions', {})
    try:
        declared = [float(dims[k]) for k in ('length_m', 'width_m', 'height_m')]
        tolerance = float(profile['dimension_tolerance_m'])
        pitch = float(attrs['pixel_pitch_m'])
        points = np.asarray(attrs['measured_surface_points'], dtype=float)
    except (KeyError, TypeError, ValueError):
        blocked('declared cube dimensions or measured surface unavailable')
    if (not profile.get('id') or profile.get('workpiece_shape') != 'cube' or
            obj.get('label') not in profile.get('perception_labels', []) or
            not all(math.isfinite(v) and v > 0 for v in declared) or
            max(declared)-min(declared) > 1e-9 or not math.isfinite(tolerance) or
            not 0 < tolerance <= .0005 or not math.isfinite(pitch) or not 0 < pitch <= .002):
        blocked('explicit compatible cube profile and bounded dimension/pixel uncertainty required')
    if (attrs.get('measurement_frame') != 'world' or
            attrs.get('position_semantics') != 'visible_surface_centroid' or
            points.ndim != 2 or points.shape[1] != 3 or len(points) < 100 or
            not np.isfinite(points).all()):
        blocked('finite segmented surface in verified world frame required')
    side = declared[0]
    if np.ptp(points, axis=0).max() > math.sqrt(3)*side + 2*pitch:
        blocked('surface exceeds declared cube; possible merged instance')
    # Deterministic dominant-plane consensus, followed by least-squares normal.
    rng = np.random.default_rng(0)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(64):
        a,b,c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b-a,c-a)
        length = np.linalg.norm(n)
        if length < 1e-12:
            continue
        keep = np.abs((points-a) @ (n/length)) <= .0002
        if keep.sum() > best.sum():
            best = keep
    if best.sum() < .90*len(points):
        blocked('no dominant planar face with at least 90 percent surface support')
    face = points[best]
    origin = face.mean(axis=0)
    _, singular, axes = np.linalg.svd(face-origin, full_matrices=False)
    normal = axes[-1]
    if normal[2] < 0:
        normal = -normal
    residual = float(np.abs((face-origin) @ normal).max())
    if normal[2] < .7 or residual > .00025 or singular[1] < .01:
        blocked('top face normal, planarity or two-dimensional support unresolved')
    u = axes[0]
    v = np.cross(normal,u)
    planar = np.column_stack(((face-origin) @ u, (face-origin) @ v)).astype(np.float32)
    centre, sizes, angle = cv2.minAreaRect(planar)
    if min(sizes) < side-2*pitch-tolerance or max(sizes) > side+2*pitch+tolerance:
        blocked('incomplete or inconsistent face extents; expose all four cube edges')
    hull_area = cv2.contourArea(cv2.convexHull(planar))
    if hull_area < .80*side*side:
        blocked('insufficient face coverage; expose the complete top face')
    theta = math.radians(angle)
    edge = math.cos(theta)*u + math.sin(theta)*v
    other = np.cross(normal,edge)
    face_centre = origin + centre[0]*u + centre[1]*v
    # Bound unknown face yaw by all enclosing declared squares, modulo cube symmetry.
    relative = face-face_centre
    feasible = []
    for degrees in range(-45,46):
        a = math.radians(degrees)
        x = math.cos(a)*edge + math.sin(a)*other
        y = np.cross(normal,x)
        if max(np.ptp(relative @ x),np.ptp(relative @ y)) <= side + 2*pitch + 2*tolerance:
            feasible.append(abs(a))
    if not feasible or max(feasible) > math.radians(20):
        blocked('face yaw uncertainty too broad; complete edge observation required')
    angle_bound = max(feasible) + math.radians(1) + math.atan2(2*residual,side)
    centre_bound = 2*pitch + residual + tolerance/2
    padding = centre_bound + math.sqrt(3)*side*math.sin(angle_bound/2) + tolerance/2
    rotation = np.column_stack((edge,other,normal))
    rpy = [math.atan2(rotation[2,1],rotation[2,2]),
           math.atan2(-rotation[2,0],math.hypot(rotation[0,0],rotation[1,0])),
           math.atan2(rotation[1,0],rotation[0,0])]
    out = copy.deepcopy(obj)
    out['pose'] = {'frame_id':'world', 'position':(face_centre-normal*side/2).tolist(),
                   'orientation_xyzw':quaternion_from_rpy(rpy)}
    out['dimensions_xyz'] = [side+2*padding]*3
    out['shape'] = 'box'
    out['attributes']['geometry_provenance'] = {
        'profile_id':profile['id'], 'dimension_source':'declared_workpiece_profile',
        'declared_dimensions_m':declared, 'dimension_tolerance_m':tolerance,
        'position_source':'measured_face_centre_minus_measured_normal_half_declared_height',
        'face_centre_world':face_centre.tolist(), 'face_normal_world':normal.tolist(),
        'face_support_points':int(best.sum()), 'face_support_fraction':float(best.mean()),
        'face_hull_area_m2':hull_area, 'plane_max_residual_m':residual,
        'centre_uncertainty_m':centre_bound, 'orientation_uncertainty_rad':angle_bound,
        'orientation_equivalence':'cube face rotations modulo 90 degrees',
        'collision_padding_m':padding, 'collision_dimensions_source':'conservative_pose_envelope'}
    return out


def qualify_simulation_dimensions(snapshot, profile, authored_world, expected_sha256, model_names):
    """Bind a uniform simulation-only BOX inventory to an authored specification.

    Loaded collision geometry is read by capture, never object poses. This is
    model specification authority, NOT metrology, support contact or execution
    authority. Existing pose bounds and collision envelopes remain untouched.
    """
    import hashlib
    import xml.etree.ElementTree as ET

    def blocked(reason):
        raise ValueError('BLOCKED: simulation dimensions: ' + reason)

    source = snapshot.get('source', {})
    stamp = snapshot.get('timestamp')
    clock = source.get('acquisition_clock_ns')
    loaded = source.get('simulation_asset_geometry', {})
    query_clock = loaded.get('query_clock_ns')
    intrinsics = source.get('intrinsics')
    if (not isinstance(intrinsics, list) or len(intrinsics) != 4 or
            not all(type(v) in (int, float) and math.isfinite(v) for v in intrinsics) or
            min(intrinsics[:2]) <= 0 or source.get('camera_pose_source') != 'live_scene_info' or
            source.get('camera_is_static') is not True or
            source.get('camera_definition_source') != 'live_generate_world_sdf'):
        blocked('qualified calibration and static camera transform required')
    if (snapshot.get('frame_id') != 'world' or source.get('clock_domain') != 'gazebo_simulation' or
            type(stamp) is not int or stamp <= 0 or type(clock) is not int or
            not 0 <= clock-stamp <= 1000000000 or
            any(source.get(k) != stamp for k in ('rgb_stamp_ns','depth_stamp_ns','info_stamp_ns')) or
            loaded.get('source') != 'live_generate_world_sdf' or loaded.get('capture_stamp_ns') != stamp or
            type(query_clock) is not int or type(stamp) is not int or
            not 0 <= query_clock-stamp <= 1000000000):
        blocked('fresh synchronized simulation capture and loaded geometry required')
    digest = hashlib.sha256(authored_world).hexdigest()
    if digest != expected_sha256:
        blocked('authored world hash mismatch')
    if (not model_names or any(not isinstance(n, str) or not n for n in model_names) or
            len(set(model_names)) != len(model_names)):
        blocked('explicit unique closed-world workpiece inventory required')
    try:
        root = ET.fromstring(authored_world)
        worlds = root.findall('world')
        if len(worlds) != 1:
            blocked('one authored world required')
        world = worlds[0]
        if world.get('name') != loaded.get('world') or world.findall('include'):
            blocked('world identity mismatch or unresolved include')
        authored, authored_inventory = [], []
        for model in world.findall('model'):
            static = model.findtext('static', 'false').strip()
            if static not in ('true', 'false', '0', '1'):
                blocked('invalid static declaration')
            authored_inventory.append((model.get('name'), static in ('true', '1')))
            if static in ('true', '1'):
                continue
            links = model.findall('link')
            if (len(links) != 1 or model.findall('model') or model.findall('include') or
                    model.findall('joint') or model.findall('plugin')):
                blocked('only explicit rigid single-link workpieces supported')
            collisions = links[0].findall('collision')
            if len(collisions) != 1:
                blocked('one collision BOX per workpiece required')
            shape = collisions[0].find('geometry')
            if shape is None or len(shape) != 1 or shape[0].tag != 'box':
                blocked('collision BOX required')
            dimensions = [float(v) for v in shape.findtext('box/size', '').split()]
            if len(dimensions) != 3 or not all(math.isfinite(v) and v > 0 for v in dimensions):
                blocked('finite positive BOX dimensions required')
            authored.append(dict(name=model.get('name'), static=False, supported=True,
                link=links[0].get('name'), collision=collisions[0].get('name'), dimensions_m=dimensions))
        models = loaded.get('models', [])
        if (loaded.get('complete') is not True or not isinstance(models, list) or
                any(not isinstance(m, dict) or type(m.get('static')) is not bool for m in models)):
            blocked('complete loaded inventory required')
        actual_inventory = [(m.get('name'), m['static']) for m in models]
        if (any(not isinstance(n, str) or not n for n, _ in actual_inventory+authored_inventory) or
                len({n for n, _ in actual_inventory}) != len(actual_inventory) or
                len({n for n, _ in authored_inventory}) != len(authored_inventory) or
                sorted(actual_inventory) != sorted(authored_inventory)):
            blocked('loaded world model inventory differs from authored world')
        dynamic = [m for m in models if not m['static']]
        names = [m.get('name') for m in dynamic]
        if (len(names) != len(set(names)) or set(names) != set(model_names) or
                len(authored) != len(model_names) or {m['name'] for m in authored} != set(model_names)):
            blocked('unknown, missing or duplicate workpiece')
        for a in authored:
            actual = next(m for m in dynamic if m['name'] == a['name'])
            if any(actual.get(k) != v for k, v in a.items()):
                blocked('loaded collision geometry differs from authored specification')
        dimensions = [float(profile['dimensions'][k]) for k in ('length_m','width_m','height_m')]
        if (not profile.get('id') or profile.get('workpiece_shape') != 'cube' or
                max(dimensions)-min(dimensions) > 1e-12 or
                any(a['dimensions_m'] != dimensions for a in authored)):
            blocked('explicit profile does not match uniform loaded cube inventory')
        objects = snapshot.get('objects', [])
        ids = [o.get('object_id') for o in objects]
        if (len(objects) > len(model_names) or len(ids) != len(set(ids)) or
                any(not i for i in ids) or
                any(o.get('label') not in profile.get('perception_labels', []) for o in objects)):
            blocked('observations do not fit explicitly associated inventory')
    except (ET.ParseError, KeyError, TypeError, AttributeError, ValueError) as exc:
        if str(exc).startswith('BLOCKED:'):
            raise
        blocked('invalid specification: ' + str(exc))
    out = copy.deepcopy(snapshot)
    spec = dict(scope='simulation_only_replay', source_world_sha256=digest,
        loaded_geometry_source=loaded['source'], world=loaded['world'], capture_stamp_ns=stamp,
        profile_id=profile['id'], inventory=authored, dimensions_m=dimensions,
        association='explicit uniform inventory/profile; not simulator instance identification',
        numeric_error_m=math.nextafter(2*max(math.ulp(v) for v in dimensions), math.inf),
        numeric_error_scope='dimension decimal/binary conversion only; not pose, depth or physics error',
        physical_metrology=False, contact_authority=False)
    for obj in out['objects']:
        obj.setdefault('attributes', {})['simulation_dimension_specification'] = copy.deepcopy(spec)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-pose', type=float, nargs=6)
    parser.add_argument('--workpiece-profile', type=Path, help='Explicit declared cube profile; offline reconstruction only')
    parser.add_argument('--replay-output', type=Path, help='Existing detected_objects/v1 input for plan-only acceptance')
    parser.add_argument('--simulation-world', type=Path, help='Optional immutable authored simulation world; no pose authority')
    parser.add_argument('--simulation-world-sha256', help='Independently pinned authored world SHA256')
    parser.add_argument('--simulation-workpieces', nargs='+', help='Explicit uniform profile-associated model inventory')
    parser.add_argument('--support-depth', type=Path, help='Optional same-capture float32 depth for relative support diagnostics')
    parser.add_argument('--support-id', help='Loaded static support identity; ROI association remains unqualified')
    parser.add_argument('--support-roi', type=int, nargs=4, help='Explicit table pixel ROI: u0 v0 u1 v1; diagnostics only')
    args = parser.parse_args()
    if args.workpiece_profile and args.camera_pose is None:
        parser.error('BLOCKED: reconstruction requires --camera-pose and optical input for verified extrinsics')
    snapshot = json.loads(args.snapshot.read_text())
    if args.camera_pose is not None:
        snapshot = transform_snapshot(snapshot, args.camera_pose)
    if args.replay_output and not args.workpiece_profile:
        parser.error('--replay-output requires --workpiece-profile')
    simulation_args = (args.simulation_world, args.simulation_world_sha256, args.simulation_workpieces)
    if any(simulation_args) and (not all(simulation_args) or not args.workpiece_profile):
        parser.error('simulation specification requires world, pinned SHA256, inventory and workpiece profile')
    support_args = (args.support_depth, args.support_id, args.support_roi)
    if any(support_args) and (not all(support_args) or not all(simulation_args)):
        parser.error('relative support diagnostics require depth, identity, ROI and simulation dimension qualification')
    if args.workpiece_profile:
        import hashlib
        import yaml
        profile_data = yaml.safe_load(args.workpiece_profile.read_text())
        if profile_data.get('schema_version') != 'environment_asset/v1':
            raise ValueError('BLOCKED: existing environment_asset/v1 profile required')
        source = snapshot.get('source', {})
        clock,stamp = source.get('acquisition_clock_ns'),snapshot.get('timestamp')
        if (snapshot.get('frame_id') != 'world' or source.get('clock_domain') != 'gazebo_simulation' or
                type(clock) is not int or type(stamp) is not int or stamp <= 0 or
                not 0 <= clock-stamp <= 1000000000 or
                any(source.get(k) != stamp for k in ('rgb_stamp_ns','depth_stamp_ns','info_stamp_ns'))):
            raise ValueError('BLOCKED: verified frame and fresh simulation-clock acquisition evidence required')
        accepted, rejected = [], []
        for obj in snapshot['objects']:
            try:
                item = estimate_cube_geometry(obj,profile_data['asset'])
                item['attributes']['geometry_provenance']['profile_sha256'] = hashlib.sha256(args.workpiece_profile.read_bytes()).hexdigest()
                accepted.append(item)
            except ValueError as exc:
                rejected.append({'object_id':obj['object_id'],'reason':str(exc)})
        snapshot['objects'] = accepted
        source['geometry_rejected_objects'] = rejected
        if args.simulation_world:
            snapshot = qualify_simulation_dimensions(snapshot, profile_data['asset'],
                args.simulation_world.read_bytes(), args.simulation_world_sha256, args.simulation_workpieces)
    if args.support_depth:
        import numpy as np
        from stage_a_relative_support import measure_relative_support
        source = snapshot['source']
        depth_bytes = args.support_depth.read_bytes()
        depth = np.frombuffer(depth_bytes, dtype=np.float32).reshape(source['height'], source['width'])
        report = measure_relative_support(snapshot, depth, args.support_roi, args.support_id)
        source['relative_support_diagnostic'] = {k:v for k,v in report.items() if k != 'objects'}
        source['relative_support_diagnostic']['depth_sha256'] = hashlib.sha256(depth_bytes).hexdigest()
        for obj, measurement in zip(snapshot['objects'], report['objects']):
            obj['attributes']['relative_support_diagnostic'] = measurement
    errors = validate_normalized_snapshot(snapshot, expected_scene_id='ur5_2f_test', expected_camera_id='stage_a_camera')
    if errors:
        raise ValueError('; '.join(errors))
    with args.output.open('x') as output:
        json.dump(snapshot, output, indent=2, allow_nan=False)
    if args.replay_output:
        replay = {'schema_version':'detected_objects/v1','scene_id':snapshot['scene_id'],
                  'source':{'mode':'replayed_snapshot','plan_only':True,'frozen_capture_timestamp_ns':snapshot['timestamp'],
                            'geometry_scope':'accepted observations only; scene completeness unverified',
                            'clock_domain':'gazebo_simulation','geometry_snapshot':str(args.output)},'objects':[]}
        for obj in snapshot['objects']:
            q = obj['pose']['orientation_xyzw']
            x,y,z,w = q
            rpy = [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
                   math.asin(max(-1,min(1,2*(w*y-z*x)))),
                   math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
            replay['objects'].append({'object_id':obj['object_id'],'class_id':obj['label'],
                'confidence':obj['confidence'],'timestamp':0,
                'pose':{'frame_id':'world','xyz':obj['pose']['position'],'rpy':rpy},
                'dimensions':obj['dimensions_xyz'],'attributes':obj['attributes']})
        with args.replay_output.open('x') as output:
            json.dump(replay,output,indent=2,allow_nan=False)
    status = 'BLOCKED' if args.workpiece_profile and not snapshot['objects'] else 'PASS'
    print(f"{status}: {len(snapshot['objects'])} normalized observations in {snapshot['frame_id']}")
    if args.support_depth:
        print('BLOCKED: support contact; qualified total relative error bound missing (geometry export retained)')
        raise SystemExit(2)
    if status == 'BLOCKED':
        raise SystemExit(2)

if __name__ == '__main__':
    main()
