"""Exact binary-input geometry tests; synthetic records are not runtime evidence."""
import copy
import sys
from fractions import Fraction
from pathlib import Path

import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def pair(z=.012499):
    return dict(collision_id=42,collision_name='a0::part_00::link::collision',
        model_name='part_00',shape='BOX',dimensions_m=[.025]*3,
        centre_world=[0.,0.,z],axes_world=np.eye(3).tolist(),
        support_collision_id=7,support_collision_name='a0::pick_support::support_link::support_collision',
        support_shape='PLANE',support_point_world=[0.,0.,0.],support_normal_world=[0.,0.,1.],
        backend_state_error_bound_m=None,manifold_points=[])


def test_complete_box_bound_does_not_depend_on_manifold():
    from stage_a_physics_contact import enclose_box_plane
    p=pair();r=enclose_box_plane(p)
    exact=Fraction.from_float(p['centre_world'][2])-Fraction.from_float(.025)/2
    assert Fraction.from_float(r['minimum_gap_interval_m'][0]) <= exact <= Fraction.from_float(r['minimum_gap_interval_m'][1])
    assert r['conditional_penetration_upper_m'] >= 1e-6
    assert r['physical_penetration_upper_m'] is None
    assert r['decision']=='BLOCKED_BACKEND_STATE_ERROR'
    assert r['evaluated_corner_count']==8
    assert p['manifold_points']==[]


def test_deep_box_with_shallow_reported_manifold_rejects():
    from stage_a_physics_contact import enclose_box_plane
    p=pair(.012);p['manifold_points']=[{'depth_m':0.}]
    r=enclose_box_plane(p)
    assert r['conditional_penetration_upper_m']>.0001
    assert r['decision']=='BLOCKED_EXCESSIVE_PENETRATION'


def test_rotated_cube_uses_every_corner():
    from stage_a_physics_contact import enclose_box_plane
    p=pair();c,s=float(np.cos(.1)),float(np.sin(.1))
    p['axes_world']=[[1.,0.,0.],[0.,c,-s],[0.,s,c]]
    assert enclose_box_plane(p)['conditional_penetration_upper_m']>.001


@pytest.mark.parametrize('bad',['missing','nan','shape','rotation','plane','support_id','dimension'])
def test_bad_or_unsupported_measurement_rejects(bad):
    from stage_a_physics_contact import enclose_box_plane
    p=pair()
    if bad=='missing': p.pop('axes_world')
    if bad=='nan': p['centre_world'][0]=float('nan')
    if bad=='shape': p['shape']='MESH'
    if bad=='rotation': p['axes_world']=[[1.,1.,0.],[0.,1.,0.],[0.,0.,1.]]
    if bad=='plane': p['support_normal_world']=[.1,0.,1.]
    if bad=='support_id': p['support_collision_id']=42
    if bad=='dimension': p['dimensions_m'][0]=-1.
    with pytest.raises(ValueError,match='BLOCKED'):
        enclose_box_plane(p)


def binding_inputs():
    p=pair()
    record=dict(schema='workcell_physics_contact_state/v1',run_id='session-a',world='a0',
        step=100,stamp_ns=100000000,dt_ns=1000000,frame_id='world',
        support_collision_id=7,support_collision_name=p['support_collision_name'],
        state_source='Fortress_PostUpdate_ECM',complete=True,pairs=[p])
    snapshot=dict(timestamp=100000000,frame_id='stage_a_camera_optical_frame',source={
        'clock_domain':'gazebo_simulation','acquisition_clock_ns':100000000,'width':40,'height':40,
        'physics_measurement':dict(run_id='session-a',world='a0',step=100,stamp_ns=100000000,
            support_collision_id=7,support_collision_name=p['support_collision_name']),
        'simulation_asset_geometry':dict(world='a0',capture_stamp_ns=100000000,complete=True,
            models=[dict(name='part_00',static=False,supported=True,link='link',collision='collision',dimensions_m=[.025]*3),
                    dict(name='pick_support',static=True)]),
        'camera_is_static':True,'camera_pose_source':'live_scene_info',
        'camera_definition_source':'live_generate_world_sdf',
        'camera_world_pose':[0.,0.,1.,0.,2**-.5,0.,2**-.5],
        'rgb_stamp_ns':100000000,'depth_stamp_ns':100000000,'info_stamp_ns':100000000,
        'intrinsics':[100.,100.,20.,20.]},
        objects=[dict(object_id='epd_100000000_0',attributes={'mask_file':'mask_0.png'})])
    mask=np.zeros((40,40),np.uint8);mask[19:22,19:22]=255
    return record,snapshot,{'epd_100000000_0':mask}


def test_binding_preserves_epd_and_never_promotes_nominal_projection():
    from stage_a_physics_contact import bind_contact_observations
    r,s,m=binding_inputs();before=copy.deepcopy(s)
    out=bind_contact_observations(r,s,m)
    assert s==before
    assert out['pairs'][0]['candidate_object_ids']==['epd_100000000_0']
    assert out['pairs'][0]['binding']=='BLOCKED_PROJECTION_UNQUALIFIED'
    assert out['contact_authority'] is False
    assert out['decision']=='BLOCKED'
    assert 'centre_world' not in out['pairs'][0]


@pytest.mark.parametrize('bad',['stamp','run','world','incomplete','identity','omitted','frame','camera','mask','ambiguous',
    'support_name','support_numeric_id','stale','mask_shape','snapshot_frame'])
def test_binding_rejects_bad_evidence(bad):
    from stage_a_physics_contact import bind_contact_observations
    r,s,m=binding_inputs()
    if bad=='stamp': r['stamp_ns']+=1000000
    if bad=='run': r['run_id']='other'
    if bad=='world': r['world']='other'
    if bad=='incomplete': r['complete']=False
    if bad=='identity': r['pairs'][0]['collision_name']='wrong'
    if bad=='omitted': r['pairs']=[]
    if bad=='frame': r['frame_id']='camera'
    if bad=='camera': s['source']['camera_is_static']=False
    if bad=='mask': m={}
    if bad=='support_name': r['pairs'][0]['support_collision_name']='a0::pick_support::wrong::wrong'
    if bad=='support_numeric_id': r['pairs'][0]['support_collision_id']=99
    if bad=='stale': s['source']['acquisition_clock_ns']+=1000000001
    if bad=='mask_shape': m['epd_100000000_0']=m['epd_100000000_0'][:-1]
    if bad=='snapshot_frame': s['frame_id']='wrong'
    if bad=='ambiguous':
        s['objects'].append(dict(object_id='epd_100000000_1',attributes={'mask_file':'mask_1.png'}))
        m['epd_100000000_1']=m['epd_100000000_0'].copy()
    with pytest.raises(ValueError,match='BLOCKED'):
        bind_contact_observations(r,s,m)


def test_backend_scalar_override_cannot_qualify_state():
    from stage_a_physics_contact import enclose_box_plane
    p=pair();p['backend_state_error_bound_m']=0.
    assert enclose_box_plane(p)['physical_penetration_upper_m'] is None


def test_cli_retains_explicit_failure_report_for_mismatched_step(tmp_path):
    import json
    import subprocess
    import cv2
    r,s,m=binding_inputs();r['stamp_ns']+=1
    (tmp_path/'physics_measurement.json').write_text(json.dumps(r))
    (tmp_path/'snapshot.json').write_text(json.dumps(s))
    cv2.imwrite(str(tmp_path/'mask_0.png'),m['epd_100000000_0'])
    output=tmp_path/'report.json'
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/stage_a_physics_contact.py'),
        str(tmp_path),'--output',str(output)],capture_output=True,text=True)
    assert result.returncode==2
    report=json.loads(output.read_text())
    assert report['decision']=='BLOCKED'
    assert report['contact_authority'] is False
    assert 'step' in report['failure_reason']
