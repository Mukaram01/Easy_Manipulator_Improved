"""Measured-face reconstruction must not turn a partial surface into hidden truth."""
import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from stage_a_rgbd_snapshot import estimate_cube_geometry
from perceived_object_grasp_plan import quaternion_from_rpy, rotate_vector

PROFILE = {'id': 'stage_a_cube_25mm', 'family': 'custom', 'workpiece_shape': 'cube',
           'dimensions': {'length_m': .025, 'width_m': .025, 'height_m': .025},
           'perception_labels': ['cube'], 'dimension_tolerance_m': .00025}


def surface(tilted=False):
    q = quaternion_from_rpy([.25, -.15, .4] if tilted else [0, 0, .4])
    centre = np.array([.4, -.2, .08])
    pts = [centre + rotate_vector(q, [x, y, .0125])
           for x in np.linspace(-.0125, .0125, 23)
           for y in np.linspace(-.0125, .0125, 23)]
    obj = {'object_id': 'epd_100_0', 'label': 'cube', 'confidence': .95,
           'centroid': np.mean(pts, axis=0).tolist(),
           'attributes': {'position_semantics': 'visible_surface_centroid',
                          'measured_surface_points': np.array(pts).tolist(),
                          'measurement_frame': 'world', 'pixel_pitch_m': .0006}}
    return obj, centre, q


@pytest.mark.parametrize('tilted', [False, True])
def test_full_measured_face_estimates_centre_along_normal(tilted):
    obj, centre, q = surface(tilted)
    original = copy.deepcopy(obj)
    result = estimate_cube_geometry(obj, PROFILE)
    assert result['centroid'] == original['centroid']
    assert result['pose']['position'] == pytest.approx(centre, abs=1e-5)
    normal = rotate_vector(result['pose']['orientation_xyzw'], [0, 0, 1])
    assert normal == pytest.approx(rotate_vector(q, [0, 0, 1]), abs=1e-5)
    assert all(v > .025 for v in result['dimensions_xyz'])
    evidence = result['attributes']['geometry_provenance']
    assert evidence['dimension_source'] == 'declared_workpiece_profile'
    assert evidence['declared_dimensions_m'] == [.025]*3
    assert evidence['profile_id'] == PROFILE['id']
    assert evidence['centre_uncertainty_m'] > 0
    assert obj == original


@pytest.mark.parametrize('bad', ['partial', 'merged', 'nonplanar', 'frame', 'profile', 'missing', 'nan'])
def test_unsupported_geometry_stays_blocked(bad):
    obj, _, _ = surface()
    profile = copy.deepcopy(PROFILE)
    if bad == 'partial': obj['attributes']['measured_surface_points'] = obj['attributes']['measured_surface_points'][:100]
    if bad == 'merged':
        obj['attributes']['measured_surface_points'] += [[p[0]+.025,p[1],p[2]] for p in obj['attributes']['measured_surface_points']]
    if bad == 'nonplanar':
        for i, p in enumerate(obj['attributes']['measured_surface_points']): p[2] += .005*(i%2)
    if bad == 'frame': obj['attributes']['measurement_frame'] = 'unknown'
    if bad == 'profile': profile['dimensions'] = {}
    if bad == 'missing': obj['attributes'].pop('measured_surface_points')
    if bad == 'nan': obj['attributes']['measured_surface_points'][0][0] = float('nan')
    with pytest.raises(ValueError, match='BLOCKED'):
        estimate_cube_geometry(obj, profile)


def test_plan_only_geometry_replay_cannot_request_execution():
    from runtime_pick_inputs import replay_snapshot
    template = {'source': {'mode': 'replayed_snapshot', 'plan_only': True}, 'objects': []}
    with pytest.raises(ValueError, match='plan-only'):
        replay_snapshot(template, 100, execution_requested=True)
    assert replay_snapshot(template, 100)['source']['plan_only'] is True


def test_plan_only_geometry_rejected_even_without_replay_restamping():
    from runtime_pick_inputs import normalize
    import perceived_object_grasp_plan as geometry
    template = {'schema_version':'detected_objects/v1',
                'source':{'plan_only':True}, 'objects':[]}
    with pytest.raises(ValueError, match='plan-only'):
        normalize(template,100,geometry,execution_requested=True)


@pytest.mark.parametrize('bad', ['stale', 'future', 'mismatched_stamp', 'unknown_frame'])
def test_reconstruction_cli_rejects_unqualified_acquisition(tmp_path, bad):
    import json
    import subprocess
    root = Path(__file__).resolve().parents[1]
    source = {'camera_pose_source':'live_scene_info','camera_is_static':True,
              'camera_definition_source':'live_generate_world_sdf',
              'camera_world_pose':[.4,-.2,.6,0,2**-.5,0,2**-.5],
              'clock_domain':'gazebo_simulation','acquisition_clock_ns':1000000000,
              'rgb_stamp_ns':1000000000,'depth_stamp_ns':1000000000,'info_stamp_ns':1000000000}
    snap = {'schema_version':'workcell_perception_snapshot/v1','scene_id':'ur5_2f_test',
            'camera_id':'stage_a_camera','frame_id':'stage_a_camera_optical_frame',
            'timestamp':1000000000,'source':source,'objects':[]}
    if bad == 'stale': source['acquisition_clock_ns'] += 2000000000
    if bad == 'future': source['acquisition_clock_ns'] -= 1
    if bad == 'mismatched_stamp': source['depth_stamp_ns'] += 1
    if bad == 'unknown_frame': snap['frame_id'] = 'unknown'
    path = tmp_path/'snapshot.json';path.write_text(json.dumps(snap))
    result = subprocess.run([sys.executable,str(root/'scripts/stage_a_rgbd_snapshot.py'),str(path),
        '--output',str(tmp_path/'out.json'),'--camera-pose','.4','-.2','.6','0',str(np.pi/2),'0',
        '--workpiece-profile',str(root/'catalog/capabilities/environment_assets/asset_stage_a_cube_25mm.yaml')],
        capture_output=True,text=True)
    assert result.returncode != 0
    assert 'BLOCKED' in result.stderr
    assert not (tmp_path/'out.json').exists()


def test_measured_geometry_reaches_existing_collision_contract():
    pytest.importorskip('moveit_msgs')
    from dynamic_object_planning_scene_bridge import build_collision_object
    obj,_,_ = surface(tilted=True)
    ready = estimate_cube_geometry(obj,PROFILE)
    snapshot = {'schema_version':'workcell_perception_snapshot/v1','scene_id':'test',
                'camera_id':'test','timestamp':100,'frame_id':'world','objects':[ready]}
    result = build_collision_object(snapshot,ready['object_id'],'world')
    assert result.status == 'PASS'
    assert result.collision_object.id == ready['object_id']
    assert list(result.collision_object.primitives[0].dimensions) == ready['dimensions_xyz']
    assert result.collision_object.primitive_poses[0].position.z == pytest.approx(ready['pose']['position'][2])
