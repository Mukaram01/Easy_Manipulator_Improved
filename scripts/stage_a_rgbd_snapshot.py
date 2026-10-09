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
    out['frame_id'] = 'world'
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-pose', type=float, nargs=6)
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text())
    if args.camera_pose is not None:
        snapshot = transform_snapshot(snapshot, args.camera_pose)
    errors = validate_normalized_snapshot(snapshot, expected_scene_id='ur5_2f_test', expected_camera_id='stage_a_camera')
    if errors:
        raise ValueError('; '.join(errors))
    with args.output.open('x') as output:
        json.dump(snapshot, output, indent=2, allow_nan=False)
    print(f"PASS: {len(snapshot['objects'])} normalized observations in {snapshot['frame_id']}")

if __name__ == '__main__':
    main()
