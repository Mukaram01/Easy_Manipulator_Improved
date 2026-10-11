#!/usr/bin/env python3
"""Independent acceptance only: compare saved EPD output with simulator boxes.

Requires system Ignition protobuf Python bindings generated with protoc (see manual).
Ground truth is never an input to capture, EPD inference or normalized output.
"""
import argparse
import json
import math
import itertools
from pathlib import Path
import sys
import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from perceived_object_grasp_plan import rotate_vector
from ignition.msgs.scene_pb2 import Scene
from ignition.msgs.pose_v_pb2 import Pose_V
from google.protobuf.json_format import Parse


def rotation(q):
    return np.array([rotate_vector(q, axis) for axis in np.eye(3)]).T


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--poses-before', type=Path, required=True)
    p.add_argument('--poses-after', type=Path, required=True)
    a = p.parse_args()
    before,after = Pose_V(),Pose_V()
    Parse(a.poses_before.read_text(),before)
    Parse(a.poses_after.read_text(),after)
    poses = {pose.name:pose for pose in before.pose if pose.name.startswith('part_')}
    later = {pose.name:pose for pose in after.pose if pose.name.startswith('part_')}
    settling_delta=max(math.dist([p.position.x,p.position.y,p.position.z],
        [later[n].position.x,later[n].position.y,later[n].position.z]) for n,p in poses.items())
    angular_delta=0.0
    for name,pose in poses.items():
        q,r=pose.orientation,later[name].orientation
        dot=abs(q.x*r.x+q.y*r.y+q.z*r.z+q.w*r.w)
        angular_delta=max(angular_delta,2*math.acos(min(1.0,dot)))
    if settling_delta>0.0001 or angular_delta>0.001:
        raise ValueError('moving pile: separate ground-truth samples cannot validate this capture')
    def stamp(msg):
        return msg.header.stamp.sec*1000000000+msg.header.stamp.nsec
    scene = Scene()
    scene.ParseFromString((a.capture/'validation_scene.pb').read_bytes())
    snapshot = json.loads((a.capture/'world_snapshot.json').read_text())
    if not stamp(before) <= snapshot['timestamp'] <= stamp(after):
        raise ValueError('ground-truth samples do not bracket capture timestamp')
    metadata = snapshot['source']
    fx,fy,cx,cy = metadata['intrinsics']
    height,width = metadata['height'],metadata['width']
    depth = np.fromfile(a.capture/'depth.f32',dtype=np.float32).reshape(height,width)
    cam = metadata['camera_world_pose']
    optical_to_camera = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    optical_to_world = rotation(cam[3:]) @ optical_to_camera
    v,u = np.indices((height,width))
    rays = np.stack(((u-cx)/fx,(v-cy)/fy,np.ones_like(u)),axis=-1) @ optical_to_world.T
    boxes = []
    for model in scene.model:
        if not model.name.startswith('part_'):
            continue
        pose = poses[model.name]
        xyz = np.array([pose.position.x,pose.position.y,pose.position.z])
        q = pose.orientation
        rot = rotation([q.x,q.y,q.z,q.w])
        size = model.link[0].visual[0].geometry.box.size
        half = np.array([size.x,size.y,size.z])/2
        origin = (np.array(cam[:3])-xyz) @ rot
        direction = rays @ rot
        with np.errstate(divide='ignore',invalid='ignore'):
            near = (-half-origin)/direction
            far = (half-origin)/direction
        low = np.minimum(near,far).max(axis=-1)
        high = np.maximum(near,far).min(axis=-1)
        distance = np.where((high>=low)&(low>0),low,np.inf)
        boxes.append((model.name,xyz,rot,half,distance))
    stack = np.stack([b[4] for b in boxes])
    expected_depth = stack.min(axis=0)
    winner = stack.argmin(axis=0)
    finite = np.isfinite(expected_depth) & np.isfinite(depth)
    residual = np.abs(expected_depth[finite]-depth[finite])
    interior = np.zeros((height,width),dtype=bool)
    for i in range(len(boxes)):
        region=((winner==i)&np.isfinite(expected_depth)).astype(np.uint8)
        interior |= cv2.erode(region,np.ones((3,3),np.uint8)).astype(bool)
    interior &= finite
    interior_residual = np.abs(expected_depth[interior]-depth[interior])
    counts = {b[0]:int(((winner==i)&np.isfinite(expected_depth)).sum()) for i,b in enumerate(boxes)}
    errors = []
    for obj in snapshot['objects']:
        point = np.array(obj['centroid'])
        # Surface centroids are not volume centres; report both distances explicitly.
        choices = []
        for name,xyz,rot,half,_ in boxes:
            local = np.abs((point-xyz) @ rot)
            surface = (np.linalg.norm(np.maximum(local-half,0)) if np.any(local>half)
                       else float(np.min(half-local)))
            choices.append((surface,name,float(np.linalg.norm(point-xyz))))
        surface,name,centre = min(choices)
        errors.append({'object_id':obj['object_id'],'nearest_box':name,
                       'surface_error_m':surface,'volume_centre_distance_m':centre})
        if 'pose' in obj and 'geometry_provenance' in obj.get('attributes', {}):
            _,truth_centre,truth_rotation,half,_ = next(b for b in boxes if b[0] == name)
            estimated = np.array(obj['pose']['position'])
            estimated_rotation = rotation(obj['pose']['orientation_xyzw'])
            provenance = obj['attributes']['geometry_provenance']
            centre_error = float(np.linalg.norm(estimated-truth_centre))
            corners = np.array(list(itertools.product((-1,1),repeat=3))) * half
            world_corners = corners @ truth_rotation.T + truth_centre
            local_corners = (world_corners-estimated) @ estimated_rotation
            envelope_contains = bool(np.all(np.abs(local_corners) <= np.array(obj['dimensions_xyz'])/2))
            angles = []
            for order in itertools.permutations(range(3)):
                for signs in itertools.product((-1,1),repeat=3):
                    symmetry = np.eye(3)[:,order] @ np.diag(signs)
                    if np.linalg.det(symmetry) > 0:
                        relative = estimated_rotation.T @ truth_rotation @ symmetry
                        angles.append(math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1))))
            orientation_error = min(angles)
            dimensions_match = bool(np.all(np.abs(np.array(provenance['declared_dimensions_m'])-2*half)
                                            <= provenance['dimension_tolerance_m']))
            bounded = (centre_error <= provenance['centre_uncertainty_m'] and
                       orientation_error <= provenance['orientation_uncertainty_rad'])
            errors[-1].update(estimated_centre_error_m=centre_error,
                orientation_error_rad_mod_cube_symmetry=orientation_error,
                declared_dimensions_match_truth=dimensions_match,
                collision_envelope_contains_truth=envelope_contains, pose_errors_within_bounds=bounded)
            if not envelope_contains or not dimensions_match or not bounded:
                raise ValueError('reconstructed geometry exceeds declared uncertainty or excludes simulator box: '+name)
    report = {'purpose':'independent ground-truth validation only',
              'scene_sample_limitation':'dimensions from scene/info; poses from timestamped dynamic_pose/info bracketing capture',
              'settling_position_delta_m':settling_delta,
              'settling_orientation_delta_rad':angular_delta,
              'epd_detection_count':metadata['epd_detections_ge_0_80'],
              'valid_observation_count':len(snapshot['objects']),
              'raycast_visible_pixel_counts':counts,
              'visible_box_count':sum(n>0 for n in counts.values()),
              'fully_occluded_box_count':sum(n==0 for n in counts.values()),
              'matched_box_count':len({e['nearest_box'] for e in errors}),
              'cube_depth_pixels_compared':int(finite.sum()),
              'interior_depth_pixels_compared':int(interior.sum()),
              'interior_depth_p95_abs_error_m':float(np.quantile(interior_residual,0.95)),
              'cube_depth_median_abs_error_m':float(np.median(residual)),
              'cube_depth_p95_abs_error_m':float(np.quantile(residual,0.95)),
              'observations':errors}
    print(json.dumps(report,indent=2,allow_nan=False))

if __name__ == '__main__':
    main()
