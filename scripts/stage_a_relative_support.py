#!/usr/bin/env python3
"""Same-frame relative RGB-D support diagnostics; no planning/contact authority.

Fits an explicitly selected table ROI and uses the existing measured cube face,
pose and simulation dimension provenance. Unknown systematic errors remain
unknown: this implementation cannot issue a support certificate. Frozen input
remains replay-only. No simulator dynamic pose is consumed.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
from perceived_object_grasp_plan import rotate_vector


def _require(condition, reason):
    if not condition:
        raise ValueError('BLOCKED: relative support: ' + reason)


def _vector(value, n):
    _require(isinstance(value, list) and len(value) == n and
             all(type(v) in (int, float) and math.isfinite(v) for v in value),
             'finite vector required')
    return np.asarray(value, dtype=float)


def _rotation(q):
    q = _vector(q, 4)
    _require(abs(float(q @ q)-1) <= 1e-6, 'unit quaternion required')
    return np.column_stack([rotate_vector(q, axis) for axis in np.eye(3)])


def measure_relative_support(snapshot, depth, support_roi, support_id):
    """Fail closed with actionable diagnostics for absent/malformed evidence."""
    try:
        return _measure_relative_support(snapshot, depth, support_roi, support_id)
    except (KeyError, TypeError, AttributeError, ValueError, np.linalg.LinAlgError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith('BLOCKED:'):
            raise
        raise ValueError('BLOCKED: relative support: missing or malformed measurement: ' + str(exc)) from exc


def _measure_relative_support(snapshot, depth, support_roi, support_id):
    """Return nominal geometry and unresolved budget; never mutate input.

    ROI is an operator association, not verified surface segmentation. Residuals
    and float storage precision are diagnostics, never total sensor error bounds.
    All lengths are metres. A common rigid camera/world transform cancels from
    relative dot products, but erroneous input geometry/registration does not.
    """
    source = snapshot['source']
    stamp = snapshot['timestamp']
    clock = source.get('acquisition_clock_ns')
    _require(snapshot.get('frame_id') == 'world' and
             source.get('frame_id') == 'stage_a_camera_optical_frame' and
             source.get('clock_domain') == 'gazebo_simulation' and
             type(stamp) is int and stamp > 0 and type(clock) is int and
             0 <= clock-stamp <= 1000000000 and
             all(source.get(k) == stamp for k in ('rgb_stamp_ns','depth_stamp_ns','info_stamp_ns')),
             'synchronized fresh capture in verified frames required')
    _require(source.get('camera_is_static') is True and
             source.get('camera_pose_source') == 'live_scene_info' and
             source.get('camera_definition_source') == 'live_generate_world_sdf',
             'static camera transform provenance required')
    camera = _vector(source.get('camera_world_pose'), 7)
    optical_to_world = _rotation(camera[3:].tolist()) @ np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    loaded = source.get('simulation_asset_geometry', {})
    _require(loaded.get('source') == 'live_generate_world_sdf' and
             loaded.get('complete') is True and loaded.get('capture_stamp_ns') == stamp and
             type(loaded.get('query_clock_ns')) is int and
             0 <= loaded['query_clock_ns']-stamp <= 1000000000 and
             isinstance(support_id, str) and bool(support_id) and
             sum(m.get('name') == support_id and m.get('static') is True
                 for m in loaded.get('models', [])) == 1,
             'unique loaded static support identity required')
    width, height = source.get('width'), source.get('height')
    _require(type(width) is int and type(height) is int and width > 0 and height > 0 and
             source.get('depth_encoding') == '32FC1' and source.get('depth_units') == 'metres' and
             isinstance(depth, np.ndarray) and depth.dtype == np.float32 and
             depth.shape == (height,width), 'matching float32 axial depth required')
    intrinsics = _vector(source.get('intrinsics'), 4)
    fx,fy,cx,cy = intrinsics
    _require(fx > 0 and fy > 0, 'positive focal lengths required')
    _require(len(support_roi) == 4 and all(type(v) is int for v in support_roi), 'integer ROI required')
    u0,v0,u1,v1 = support_roi
    _require(0 <= u0 < u1 <= width and 0 <= v0 < v1 <= height, 'ROI outside depth image')
    vv,uu = np.mgrid[v0:v1,u0:u1]
    z = depth[v0:v1,u0:u1]
    _require(z.size >= 100 and np.isfinite(z).all() and np.all(z > 0),
             'at least 100 finite positive table pixels required')
    points = np.column_stack((((uu-cx)*z/fx).ravel(), ((vv-cy)*z/fy).ravel(), z.ravel()))
    origin = points.mean(axis=0)
    _,singular,axes = np.linalg.svd(points-origin, full_matrices=False)
    _require(singular[1] > 0, 'two-dimensional table support required')
    normal = axes[-1]
    if normal @ (-origin) < 0:
        normal = -normal
    residual = float(np.max(np.abs((points-origin) @ normal)))
    # Do not trim large residuals or turn a selection threshold into metrology.
    table_spacing = float(np.max(np.spacing(z)))
    results = []
    ids = [o.get('object_id') for o in snapshot['objects']]
    _require(ids and all(isinstance(i,str) and i for i in ids) and len(ids) == len(set(ids)),
             'unique observed object identities required')
    for obj in snapshot['objects']:
        attrs = obj['attributes']
        provenance = attrs['geometry_provenance']
        spec = attrs.get('simulation_dimension_specification', {})
        dims = _vector(spec.get('dimensions_m'), 3)
        _require(np.all(dims > 0) and np.ptp(dims) <= 1e-12 and
                 spec.get('scope') == 'simulation_only_replay' and
                 spec.get('capture_stamp_ns') == stamp and
                 spec.get('profile_id') == provenance.get('profile_id') and
                 spec.get('dimensions_m') == provenance.get('declared_dimensions_m') and
                 isinstance(spec.get('source_world_sha256'),str) and
                 len(spec['source_world_sha256']) == 64 and
                 type(spec.get('numeric_error_m')) in (int,float) and
                 math.isfinite(spec['numeric_error_m']) and spec['numeric_error_m'] >= 0,
                 'matching qualified cube dimensions required')
        face = (_vector(provenance.get('face_centre_world'),3)-camera[:3]) @ optical_to_world
        face_normal = _vector(provenance.get('face_normal_world'),3) @ optical_to_world
        _require(abs(float(face_normal @ face_normal)-1) <= 1e-6, 'unit face normal required')
        centre = (_vector(obj['pose'].get('position'),3)-camera[:3]) @ optical_to_world
        body_to_optical = optical_to_world.T @ _rotation(obj['pose'].get('orientation_xyzw'))
        corners = np.array(list(itertools.product((-1,1),repeat=3))) * dims/2
        corner_gaps = (corners @ body_to_optical.T + centre-origin) @ normal
        gap = float(np.min(corner_gaps))
        top_height = float((face-origin) @ normal)
        storage = math.nextafter((table_spacing + float(np.spacing(np.float32(face[2]))))/2, math.inf)
        angle = math.acos(float(np.clip(normal @ face_normal,-1,1)))
        budget = {
            'float32_storage_only':dict(bound_m=None, qualified=False,
                nominal_axial_half_ulp_sum_m=storage,
                reason='nominal axial storage resolution only; fitted face depth is not a raw sample; '
                       'samplewise rounding and projection/fit propagation unqualified'),
            'dimension_representation_only':dict(bound_m=spec['numeric_error_m'],qualified=True),
            'depth_measurement':dict(bound_m=None,qualified=False,
                reason='no worst-case differential renderer/depth bias bound'),
            'pixel_calibration_segmentation':dict(bound_m=None,qualified=False,
                reason='no bounded intrinsics, pixel correspondence or face-edge bias'),
            'plane_fit_and_roll_pitch':dict(bound_m=None,qualified=False,
                reason='residuals do not bound face/table normal or extrapolation error'),
            'camera_to_world':dict(bound_m=None,qualified=False,
                reason='common rigid transform cancels algebraically; input consistency/arithmetic unqualified'),
            'timestamp_motion':dict(bound_m=None,qualified=False,
                reason='RGB/depth/info stamps equal; no within-exposure motion bound'),
            'support_registration':dict(bound_m=None,qualified=False,
                reason='operator ROI and static model identity do not prove visual/collision plane registration'),
            'floating_point_geometry':dict(bound_m=None,qualified=False,
                reason='SVD, projection, dot products and geometric tolerances lack interval enclosure')}
        results.append(dict(object_id=obj['object_id'], support_id=support_id,
            measurement_source='same_frame_segmented_top_face_and_depth_table_roi',
            capture_stamp_ns=stamp, scope='simulation_only_replay_diagnostic',
            nominal_top_face_height_m=top_height, nominal_face_table_angle_rad=angle,
            nominal_face_optical=dict(centre_xyz=face.tolist(), normal_xyz=face_normal.tolist()),
            nominal_physical_box_optical=dict(centre_xyz=centre.tolist(),
                rotation_matrix=body_to_optical.tolist(), dimensions_m=dims.tolist()),
            nominal_corner_gaps_m=corner_gaps.tolist(),
            nominal_lowest_corner_gap_m=gap, nominal_penetration_m=max(0.,-gap),
            remaining_total_vertical_error_budget_m=.0001+gap,
            qualified_dimensions=spec, uncertainty_budget=budget,
            conditional_existing_pose_set=dict(centre_radius_m=provenance['centre_uncertainty_m'],
                orientation_radius_rad=provenance['orientation_uncertainty_rad'],
                scope='original conditional reconstruction set; not newly qualified by this measurement'),
            all_configuration_penetration_upper_bound_m=None,
            verified_contact_relationship=False, downshift_0_2mm_excluded=False,
            decision='BLOCKED', reasons=[v['reason'] for v in budget.values() if not v['qualified']]))
    return dict(schema='workcell_relative_support_diagnostic/v1', camera_id=snapshot['camera_id'],
        capture_stamp_ns=stamp, support_id=support_id, support_association='operator ROI; unverified registration',
        table_roi_uv=list(support_roi), table_pixels=int(z.size),
        table_plane_optical=dict(point=origin.tolist(),normal=normal.tolist(),max_residual_m=residual),
        contact_authority=False, decision='BLOCKED', objects=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path, help='Existing qualified world snapshot')
    parser.add_argument('--depth', type=Path, required=True)
    parser.add_argument('--support-id', required=True)
    parser.add_argument('--support-roi', type=int, nargs=4, required=True, metavar=('U0','V0','U1','V1'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text())
    depth = np.fromfile(args.depth, dtype=np.float32).reshape(snapshot['source']['height'],snapshot['source']['width'])
    result = measure_relative_support(snapshot, depth, args.support_roi, args.support_id)
    result['input_sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.snapshot,args.depth)}
    result['input_binding'] = 'operator supplied files; hashes pin replay bytes, not sensor authenticity'
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('BLOCKED: relative measurement exported; qualified total error bound missing')
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
