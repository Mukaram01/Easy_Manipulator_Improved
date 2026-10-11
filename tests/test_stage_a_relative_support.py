"""Relative metrology is a measurement diagnostic, never implicit contact authority."""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))


def inputs():
    # Synthetic geometry for unit tests ONLY, no runtime certification evidence.
    stamp = 100
    z = np.full((40, 40), .6, dtype=np.float32)
    snapshot = {'frame_id':'world', 'timestamp':stamp, 'camera_id':'camera',
        'source':{'frame_id':'stage_a_camera_optical_frame', 'width':40, 'height':40,
            'depth_encoding':'32FC1', 'depth_units':'metres', 'intrinsics':[100.,100.,20.,20.],
            'clock_domain':'gazebo_simulation', 'acquisition_clock_ns':stamp,
            'rgb_stamp_ns':stamp, 'depth_stamp_ns':stamp, 'info_stamp_ns':stamp,
            'camera_world_pose':[0.,0.,.6,0.,2**-.5,0.,2**-.5],
            'camera_is_static':True, 'camera_pose_source':'live_scene_info',
            'camera_definition_source':'live_generate_world_sdf',
            'simulation_asset_geometry':{'source':'live_generate_world_sdf',
                'capture_stamp_ns':stamp,'query_clock_ns':stamp,'complete':True,
                'models':[{'name':'table','static':True}]}},
        'objects':[{'object_id':'cube','pose':{'position':[0.,0.,.0125],
            'orientation_xyzw':[0.,0.,0.,1.]}, 'dimensions_xyz':[.04]*3,
            'attributes':{'geometry_provenance':{
                'face_centre_world':[0.,0.,.025], 'face_normal_world':[0.,0.,1.],
                'plane_max_residual_m':0., 'centre_uncertainty_m':.003,
                'orientation_uncertainty_rad':.2,'profile_id':'cube25',
                'declared_dimensions_m':[.025]*3},
                'simulation_dimension_specification':{'capture_stamp_ns':stamp,
                    'scope':'simulation_only_replay', 'profile_id':'cube25',
                    'source_world_sha256':'a'*64, 'dimensions_m':[.025]*3,
                    'numeric_error_m':1e-17}}}]}
    return snapshot, z


def measure(s, z, roi=(1,1,18,18), support='table'):
    from stage_a_relative_support import measure_relative_support
    return measure_relative_support(s, z, roi, support)


def test_same_frame_measurement_retains_unknown_budget_and_original_box():
    s,z = inputs(); before = copy.deepcopy(s)
    result = measure(s,z)
    assert s == before
    item = result['objects'][0]
    assert item['nominal_lowest_corner_gap_m'] == pytest.approx(0., abs=1e-7)
    assert item['nominal_top_face_height_m'] == pytest.approx(.025, abs=1e-7)
    assert item['decision'] == 'BLOCKED'
    assert item['verified_contact_relationship'] is False
    assert item['uncertainty_budget']['depth_measurement']['bound_m'] is None
    storage = item['uncertainty_budget']['float32_storage_only']
    assert storage['bound_m'] is None
    assert storage['qualified'] is False
    assert storage['nominal_axial_half_ulp_sum_m'] > 0
    assert item['all_configuration_penetration_upper_bound_m'] is None
    assert item['downshift_0_2mm_excluded'] is False
    assert item['conditional_existing_pose_set']['centre_radius_m'] == .003


@pytest.mark.parametrize('bad', ['stamp','stale','future','frame','transform','support',
    'dimension_stamp','dimension_profile','dimension_shape','depth_shape','nan','roi','plane',
    'missing_source','missing_objects','missing_face','missing_pose_bound','orientation'])
def test_invalid_evidence_rejected(bad):
    s,z = inputs(); roi=(1,1,18,18); support='table'
    if bad == 'stamp': s['source']['depth_stamp_ns'] += 1
    if bad == 'stale': s['source']['acquisition_clock_ns'] += 1000000001
    if bad == 'future': s['source']['acquisition_clock_ns'] -= 1
    if bad == 'frame': s['frame_id']='unknown'
    if bad == 'transform': s['source']['camera_world_pose'][6]=0
    if bad == 'support': support='unknown'
    spec=s['objects'][0]['attributes']['simulation_dimension_specification']
    if bad == 'dimension_stamp': spec['capture_stamp_ns'] += 1
    if bad == 'dimension_profile': spec['profile_id']='other'
    if bad == 'dimension_shape': spec['dimensions_m']=[.025,.025,.03]
    if bad == 'depth_shape': z=z[:-1]
    if bad == 'nan': z[:]=np.nan
    if bad == 'roi': roi=(0,0,41,10)
    if bad == 'plane': roi=(1,1,2,3)
    if bad == 'missing_source': s.pop('source')
    if bad == 'missing_objects': s.pop('objects')
    if bad == 'missing_face': s['objects'][0]['attributes'].pop('geometry_provenance')
    if bad == 'missing_pose_bound': s['objects'][0]['attributes']['geometry_provenance'].pop('centre_uncertainty_m')
    if bad == 'orientation': s['objects'][0]['pose']['orientation_xyzw']=[0.,0.,0.,0.]
    with pytest.raises(ValueError, match='BLOCKED'):
        measure(s,z,roi,support)


def test_relative_measurement_invariant_under_common_translation():
    s,z=inputs(); a=measure(s,z)['objects'][0]
    delta=np.array([10.,-20.,30.])
    s['source']['camera_world_pose'][:3]=(np.array(s['source']['camera_world_pose'][:3])+delta).tolist()
    obj=s['objects'][0]
    obj['pose']['position']=(np.array(obj['pose']['position'])+delta).tolist()
    p=obj['attributes']['geometry_provenance']
    p['face_centre_world']=(np.array(p['face_centre_world'])+delta).tolist()
    b=measure(s,z)['objects'][0]
    assert a['nominal_lowest_corner_gap_m'] == pytest.approx(b['nominal_lowest_corner_gap_m'],abs=1e-13)


def test_tilt_and_downshift_affect_physical_bottom_without_changing_envelope():
    from perceived_object_grasp_plan import quaternion_from_rpy
    s,z=inputs(); s['objects'][0]['pose']['orientation_xyzw']=quaternion_from_rpy([.02,0,0])
    item=measure(s,z)['objects'][0]
    assert item['nominal_lowest_corner_gap_m'] < -.0001
    assert item['decision']=='BLOCKED'
    s,z=inputs(); s['objects'][0]['pose']['position'][2]-=.0002
    item=measure(s,z)['objects'][0]
    assert item['nominal_lowest_corner_gap_m'] < -.0001
    assert s['objects'][0]['dimensions_xyz']==[.04]*3


def test_snapshot_cli_retains_source_and_object_diagnostic_on_blocked_exit(tmp_path):
    import hashlib
    import json
    import subprocess
    s,z=inputs()
    s.update(schema_version='workcell_perception_snapshot/v1',scene_id='ur5_2f_test',
             camera_id='stage_a_camera',frame_id='stage_a_camera_optical_frame')
    points=[[-y,-x,.575] for x in np.linspace(-.0125,.0125,23)
                           for y in np.linspace(-.0125,.0125,23)]
    s['objects']=[dict(object_id='cube',label='cube',confidence=.95,
        centroid=np.mean(points,axis=0).tolist(),attributes=dict(surface_points_optical=points,
        position_semantics='visible_surface_centroid',pixel_pitch_m=.0006))]
    loaded=s['source']['simulation_asset_geometry']
    loaded['world']='test'
    loaded['models'].append(dict(name='cube',static=False,supported=True,link='body',
        collision='shape',dimensions_m=[.025]*3))
    world=b'<sdf version="1.8"><world name="test"><model name="table"><static>true</static></model><model name="cube"><link name="body"><collision name="shape"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model></world></sdf>'
    p=tmp_path/'raw.json';p.write_text(json.dumps(s))
    d=tmp_path/'depth.f32';z.tofile(d)
    w=tmp_path/'world.sdf';w.write_bytes(world)
    out=tmp_path/'snapshot.json';replay=tmp_path/'replay.json'
    root=Path(__file__).resolve().parents[1]
    run=subprocess.run([sys.executable,str(root/'scripts/stage_a_rgbd_snapshot.py'),str(p),
        '--output',str(out),'--camera-pose','0','0','.6','0',str(np.pi/2),'0',
        '--workpiece-profile',str(root/'catalog/capabilities/environment_assets/asset_stage_a_cube_25mm.yaml'),
        '--simulation-world',str(w),'--simulation-world-sha256',hashlib.sha256(world).hexdigest(),
        '--simulation-workpieces','cube','--support-depth',str(d),'--support-id','table',
        '--support-roi','1','1','18','18','--replay-output',str(replay)],capture_output=True,text=True)
    assert run.returncode==2,run.stderr
    exported=json.loads(out.read_text())
    diagnostic=exported['source']['relative_support_diagnostic']
    assert diagnostic['table_pixels']==289
    assert diagnostic['decision']=='BLOCKED'
    assert diagnostic['depth_sha256']==hashlib.sha256(d.read_bytes()).hexdigest()
    assert exported['objects'][0]['attributes']['relative_support_diagnostic']['decision']=='BLOCKED'
    assert exported['objects'][0]['dimensions_xyz'][2]>.025
    assert json.loads(replay.read_text())['source']['plan_only'] is True
