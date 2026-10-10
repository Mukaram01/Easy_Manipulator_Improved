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


# Simulation dimensions may qualify a model specification, never contact or pose.
def simulation_dimension_inputs():
    import hashlib
    xml = '<sdf version="1.8"><world name="test"><model name="cube"><link name="body"><collision name="shape"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model></world></sdf>'
    snapshot = {'frame_id':'world', 'timestamp':100, 'objects':[surface()[0]],
        'source':{'clock_domain':'gazebo_simulation', 'acquisition_clock_ns':100,
                  'intrinsics':[500.,500.,256.,256.], 'camera_pose_source':'live_scene_info',
                  'camera_is_static':True,'camera_definition_source':'live_generate_world_sdf',
                  'rgb_stamp_ns':100,'depth_stamp_ns':100,'info_stamp_ns':100,
                  'simulation_asset_geometry':{'source':'live_generate_world_sdf',
                      'world':'test','capture_stamp_ns':100,'query_clock_ns':100,'complete':True,
                      'models':[{'name':'cube','static':False,'supported':True,
                                 'link':'body','collision':'shape','dimensions_m':[.025]*3}]}}}
    return snapshot, xml.encode(), hashlib.sha256(xml.encode()).hexdigest()


def test_simulation_dimension_spec_preserves_envelope_and_has_no_contact_authority():
    import stage_a_rgbd_snapshot as rgbd
    assert hasattr(rgbd, 'qualify_simulation_dimensions'), 'simulation dimension qualification missing'
    snapshot, xml, digest = simulation_dimension_inputs()
    snapshot['objects'] = [rgbd.estimate_cube_geometry(snapshot['objects'][0], PROFILE)]
    before = copy.deepcopy(snapshot)
    result = rgbd.qualify_simulation_dimensions(snapshot, PROFILE, xml, digest, ['cube'])
    assert result['objects'][0]['pose'] == before['objects'][0]['pose']
    assert result['objects'][0]['dimensions_xyz'] == before['objects'][0]['dimensions_xyz']
    spec = result['objects'][0]['attributes']['simulation_dimension_specification']
    assert spec['dimensions_m'] == [.025]*3
    assert spec['scope'] == 'simulation_only_replay'
    assert spec['source_world_sha256'] == digest
    assert spec['contact_authority'] is False
    assert spec['physical_metrology'] is False
    assert spec['dimensions_m'][2] + spec['numeric_error_m'] < .02525
    assert snapshot == before


@pytest.mark.parametrize('bad', ['hash','source_geometry','loaded_geometry','unknown_inventory',
    'profile','real_clock','stale','missing_calibration','missing_loaded','count','unsupported','duplicate',
    'loaded_stale','wrong_link','unknown_label','missing_inventory','nested_source','changed_matching_geometry',
    'unknown_static','missing_intrinsics','unknown_transform'])
def test_simulation_spec_fails_closed(bad):
    import stage_a_rgbd_snapshot as rgbd
    assert hasattr(rgbd, 'qualify_simulation_dimensions'), 'simulation dimension qualification missing'
    snapshot, xml, digest = simulation_dimension_inputs()
    profile = copy.deepcopy(PROFILE)
    loaded = snapshot['source']['simulation_asset_geometry']
    if bad == 'hash': digest = '0'*64
    if bad == 'source_geometry': xml = xml.replace(b'0.025 0.025 0.025', b'0.025 0.025 0.026')
    if bad == 'loaded_geometry': loaded['models'][0]['dimensions_m'][2] = .026
    if bad == 'unknown_inventory': loaded['models'].append(dict(loaded['models'][0], name='unknown'))
    if bad == 'profile': profile['dimensions']['height_m'] = .026
    if bad == 'real_clock': snapshot['source']['clock_domain'] = 'ros_wall'
    if bad == 'stale': snapshot['source']['acquisition_clock_ns'] += 1000000001
    if bad == 'missing_calibration': snapshot['source'].pop('info_stamp_ns')
    if bad == 'missing_loaded': snapshot['source'].pop('simulation_asset_geometry')
    if bad == 'count': snapshot['objects'] *= 2
    if bad == 'unsupported': loaded['models'][0]['supported'] = False
    if bad == 'duplicate': loaded['models'] *= 2
    if bad == 'loaded_stale': loaded['query_clock_ns'] += 1000000001
    if bad == 'wrong_link': loaded['models'][0]['link'] = 'other'
    if bad == 'unknown_label': snapshot['objects'][0]['label'] = 'unknown'
    if bad == 'missing_inventory': loaded['complete'] = False
    if bad == 'unknown_static': loaded['models'].append({'name':'unknown','static':True})
    if bad == 'missing_intrinsics': snapshot['source'].pop('intrinsics')
    if bad == 'unknown_transform': snapshot['source']['camera_is_static'] = False
    if bad == 'nested_source':
        import hashlib
        xml = xml.replace(b'<link name="body">', b'<model name="hidden"/><link name="body">')
        digest = hashlib.sha256(xml).hexdigest()
    if bad == 'changed_matching_geometry':
        import hashlib
        xml = xml.replace(b'0.025 0.025 0.025', b'0.026 0.026 0.026')
        loaded['models'][0]['dimensions_m'] = [.026]*3
        digest = hashlib.sha256(xml).hexdigest()
    with pytest.raises(ValueError, match='BLOCKED'):
        rgbd.qualify_simulation_dimensions(snapshot, profile, xml, digest, ['cube'])
