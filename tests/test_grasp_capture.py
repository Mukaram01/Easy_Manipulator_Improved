"""Synthetic fixtures, never historical or physical grasp evidence."""
import copy
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))


def case():
    from grasp_capture import CaptureModel, qualify_capture
    urdf='''<robot name="synthetic"><link name="world"/><link name="left"><collision><geometry><box size="0.004 0.04 0.04"/></geometry></collision></link><link name="right"><collision><geometry><box size="0.004 0.04 0.04"/></geometry></collision></link><joint name="close" type="prismatic"><parent link="world"/><child link="left"/><origin xyz="-0.04 0 0"/><axis xyz="1 0 0"/></joint><joint name="follower" type="prismatic"><parent link="world"/><child link="right"/><origin xyz="0.04 0 0"/><axis xyz="-1 0 0"/><mimic joint="close"/></joint></robot>'''
    model=CaptureModel(urdf, ['left','right'])
    binding=dict(scene_sha256='a'*64,observation_sha256='b'*64,robot_model_sha256=model.sha256,
                 srdf_sha256='c'*64,receipt_sha256='d'*64,candidate_id='synthetic::001',target_id='synthetic-box')
    data=dict(model=model,target=dict(id='synthetic-box',shape='BOX',frame_id='world',pose=[0,0,0,0,0,0,1],dimensions=[.02,.03,.03]),
              candidate_pose=[0,0,0,0,0,2**-.5,2**-.5],
              measured=dict(kind='MEASURED',wall_ns=1_000_000_000,
                            target_pose=[0,0,0,0,0,0,1],link_poses={'left':[-.04,0,0,0,0,0,1],'right':[.04,0,0,0,0,0,1]},joints={'close':0.,'follower':0.}),
              planned_open={'close':0.,'follower':0.},planned_closed={'close':.0285,'follower':.0285},
              binding=binding,now_ns=1_000_000_000)
    return qualify_capture,data


def test_bilateral_patch_has_depth_aperture_and_edge_margins():
    qualify,data=case();result=qualify(**data)
    assert result['accepted'], result
    assert result['claim']=='GEOMETRIC_ONLY'
    assert result['robustness']=='SAMPLED_NOT_CONTINUOUS_PROOF'
    assert len(result['samples'])>=13
    assert result['minimum_patch_radius_m']>=.001
    assert result['measured']['joints']=={'close':0.,'follower':0.}
    assert result['planned']['kind']=='PLANNED'


def test_rotated_tool_mount_uses_physical_closure_instead_of_candidate_y():
    """The TCP axes need not coincide with the mounted finger-closing axis."""
    import math
    import xml.etree.ElementTree as ET
    from grasp_capture import CaptureModel
    qualify,data=case();robot=copy.deepcopy(data['model'].robot)
    robot.append(ET.fromstring('<link name="mounted_tool"/>'))
    robot.append(ET.fromstring(
        '<joint name="mount" type="fixed"><parent link="world"/>'
        '<child link="mounted_tool"/><origin xyz="0.12 0.21 0.3" '
        f'rpy="0 0 {math.pi/4}"/></joint>'))
    for name in ('close','follower'):
        robot.find(f"joint[@name='{name}']/parent").set('link','mounted_tool')
    data['model']=CaptureModel(ET.tostring(robot,encoding='unicode'),['left','right'])
    data['binding']['robot_model_sha256']=data['model'].sha256
    rotation=[0.,0.,math.sin(math.pi/8),math.cos(math.pi/8)]
    data['target']['pose']=[.12,.21,.3]+rotation
    data['measured']['target_pose']=list(data['target']['pose'])
    for name,sign in [('left',-1),('right',1)]:
        offset=sign*.04/math.sqrt(2)
        data['measured']['link_poses'][name]=[.12+offset,.21+offset,.3]+rotation
    data['candidate_pose']=[.12,.21,.3,0.,0.,0.,1.]
    result=qualify(**data)
    assert result['accepted'],result['reason']
    axis=result['samples'][0]['closing_axis']
    assert axis['method']=='relative_fingertip_fk_open_to_closed'
    assert axis['direction_world']==pytest.approx([2**-.5,2**-.5,0.])
    assert axis['direction_target']==pytest.approx([1.,0.,0.],abs=1e-12)
    assert axis['relative_motion_m']==pytest.approx(.057)
    assert axis['face_axis']==0
    assert axis['face_angle_rad']==pytest.approx(0.,abs=1e-7)


def test_no_relative_fingertip_motion_cannot_supply_a_closing_axis():
    qualify,data=case();data['planned_open']=dict(data['planned_closed'])
    result=qualify(**data)
    assert not result['accepted']
    assert 'no relative fingertip closing motion' in result['reason']

@pytest.mark.parametrize('change,reason',[
    ('unilateral','mimic'),('edge','patch'),('depth','contact'),('aperture','aperture'),
    ('frame','frame'),('scale','contact'),('mimic','mimic'),('stale','stale'),('missing','evidence'),
    ('uncertainty','contact'),('nan','finite'),
])
def test_unsupported_or_marginal_capture_rejected(change,reason):
    qualify,data=case()
    if change=='unilateral':data['planned_closed']['follower']=0.
    if change=='edge':data['target']['pose'][1]=.0349
    if change=='depth':data['target']['pose'][2]=.05
    if change=='aperture':data['planned_open']=dict(data['planned_closed'])
    if change=='frame':data['target']['frame_id']='camera'
    if change=='scale':data['target']['dimensions']=[20.,30.,30.]
    if change=='mimic':
        data['measured']['joints']['follower']=.03
        data['measured']['link_poses']['right'][0]-=.03
    if change=='stale':data['now_ns']+=250_000_001
    if change=='missing':data['binding'].pop('scene_sha256')
    if change=='uncertainty':data['planned_closed']={'close':.02801,'follower':.02801}
    if change=='nan':data['planned_closed']['close']=float('nan')
    if change in ('edge','depth'):
        data['measured']['target_pose']=list(data['target']['pose'])
    result=qualify(**data)
    assert not result['accepted'], result
    assert reason in result['reason'].lower(),result


def test_raw_measured_mimic_discrepancy_is_retained_and_used():
    qualify,data=case();data['measured']['joints']['follower']=.0001;data['measured']['link_poses']['right'][0]-=.0001
    result=qualify(**data)
    assert result['accepted'],result
    assert result['measured']['joints']['follower']==.0001
    assert result['planned']['raw_mimic_residuals']['follower']==pytest.approx(.0001)
    assert result['planned']['evaluated_closed_joints']['follower']==pytest.approx(.0286)


def test_unknown_collision_shape_rejected():
    from grasp_capture import CaptureModel
    with pytest.raises(ValueError,match='unsupported'):
        CaptureModel('<robot><link name="world"><collision><geometry><cylinder radius="1" length="1"/></geometry></collision></link></robot>',['world','missing'])


def test_valid_pad_cannot_hide_deeply_penetrating_second_collision():
    from grasp_capture import CaptureModel
    import xml.etree.ElementTree as ET
    qualify,data=case();robot=copy.deepcopy(data['model'].robot)
    robot.find("link[@name='left']").append(ET.fromstring('''<collision><origin xyz="0.009 0 0"/><geometry><box size="0.016 0.01 0.01"/></geometry></collision>'''))
    data['model']=CaptureModel(ET.tostring(robot,encoding='unicode'),['left','right'])
    data['binding']['robot_model_sha256']=data['model'].sha256
    result=qualify(**data)
    assert not result['accepted']
    assert 'penetration' in result['reason']


def test_joint_sampling_includes_independent_tool_joints_with_explicit_units():
    qualify,data=case();result=qualify(**data)
    assert result['accepted'],result
    labels={s['sample'] for s in result['samples']}
    assert {'joint_close_-1','joint_close_1','joint_follower_-1','joint_follower_1'}<=labels
    rows={s['sample']:s for s in result['samples']}
    assert rows['joint_close_-1']['closing_axis']['relative_motion_m']==pytest.approx(.05695)
    assert rows['joint_close_1']['closing_axis']['relative_motion_m']==pytest.approx(.05705)


def test_mimic_chain_preserves_upstream_measured_discrepancy():
    from grasp_capture import CaptureModel
    import xml.etree.ElementTree as ET
    qualify,data=case();robot=copy.deepcopy(data['model'].robot)
    robot.append(ET.fromstring('<link name="intermediate"/>'))
    robot.append(ET.fromstring('<joint name="middle" type="prismatic"><parent link="world"/><child link="intermediate"/><axis xyz="1 0 0"/><mimic joint="close"/></joint>'))
    robot.find("joint[@name='follower']/mimic").set('joint','middle')
    data['model']=CaptureModel(ET.tostring(robot,encoding='unicode'),['left','right'])
    data['binding']['robot_model_sha256']=data['model'].sha256
    data['measured']['joints'].update(middle=.001,follower=.001)
    data['measured']['link_poses']['right'][0]-=.001
    data['planned_open']['middle']=0.;data['planned_closed']['middle']=.0285
    data['config']={'mimic_tolerance_m':.002}
    result=qualify(**data)
    assert not result['accepted']
    assert result['planned']['evaluated_closed_joints']['follower']==pytest.approx(.0295)


def test_unknown_target_shape_never_silently_becomes_a_box():
    qualify,data=case();data['target']['shape']='SPHERE'
    result=qualify(**data)
    assert not result['accepted'] and 'shape' in result['reason']


def test_measured_link_frame_disagreement_is_not_hidden_by_joint_fk():
    qualify,data=case()
    data['measured']['link_poses']['left'][0]+=.01
    result=qualify(**data)
    assert not result['accepted'] and 'measured frame' in result['reason']


def test_missing_measured_link_pose_is_insufficient_evidence():
    qualify,data=case();data['measured'].pop('link_poses')
    result=qualify(**data)
    assert not result['accepted']


@pytest.mark.parametrize('change',['stale','model','state','frame','joint_set'])
def test_completed_calculation_requires_fresh_unchanged_evidence(change):
    from grasp_capture import confirm_capture
    qualify,data=case();result=qualify(**data)
    current=copy.deepcopy(data['measured']);hashes=dict(data['binding']);now=data['now_ns']
    if change=='stale':now+=250_000_001
    if change=='model':hashes['srdf_sha256']='f'*64
    if change=='state':current['joints']['close']+=.001
    if change=='frame':current['link_poses']['left'][0]+=.000001
    if change=='joint_set':current['joints'].pop('close')
    confirmed=confirm_capture(result,current,hashes,now)
    assert not confirmed['accepted']
    assert confirmed['completion_measurement']==current
    assert confirmed['completion_checked_wall_ns']==now
    assert confirmed['measured']==data['measured']


def test_completion_rechecks_original_target_admission_without_restamping_geometry():
    from grasp_capture import confirm_capture
    qualify,data=case();result=qualify(**data)
    original=copy.deepcopy(result['measured']);samples=copy.deepcopy(result['samples'])
    current=copy.deepcopy(data['measured'])
    current['wall_ns']+=7_000_000_000
    current['target_pose'][0]+=.00000001  # Above numeric equality, within admission.
    confirmed=confirm_capture(result,current,data['binding'],current['wall_ns'])
    assert confirmed['accepted'],confirmed['reason']
    assert confirmed['measured']==original
    assert confirmed['samples']==samples
    assert confirmed['target_reference_pose']==data['target']['pose']
    assert confirmed['completion_measurement']==current
    assert confirmed['confirmed_wall_ns']==current['wall_ns']
    assert confirmed['completion_target_agreement']['reference']=='bound_observation'
    assert confirmed['completion_target_agreement']['translation_m']==pytest.approx(.00000001)
    assert confirmed['robustness']=='SAMPLED_NOT_CONTINUOUS_PROOF'


@pytest.mark.parametrize('phase',['entry','completion'])
@pytest.mark.parametrize('change',['translation','rotation'])
def test_target_admission_rejects_outside_original_translation_or_rotation_limit(phase,change):
    from grasp_capture import confirm_capture
    qualify,data=case()
    changed=copy.deepcopy(data['measured'])
    if change=='translation':changed['target_pose'][0]=.000250001
    else:changed['target_pose'][3:]=[0.,0.,float(np.sin(.0005)),float(np.cos(.0005))]
    if phase=='entry':
        data['measured']=changed
        result=qualify(**data)
        agreement=result['measurement_target_agreement']
    else:
        result=confirm_capture(qualify(**data),changed,data['binding'],data['now_ns'])
        agreement=result['completion_target_agreement']
        assert result['completion_measurement']==changed
    assert not result['accepted']
    assert 'target geometry changed since bound observation' in result['reason']
    assert not agreement['accepted']
    assert agreement['translation_limit_m']==.00025
    assert agreement['quaternion_distance_limit']==.00025


def test_completion_target_gate_cannot_accumulate_drift_between_measurements():
    from grasp_capture import confirm_capture
    qualify,data=case()
    data['measured']['target_pose'][0]=.0002
    result=qualify(**data)
    assert result['accepted'],result['reason']
    current=copy.deepcopy(data['measured']);current['target_pose'][0]=.0004
    assert current['target_pose'][0]-data['measured']['target_pose'][0]<.00025
    result=confirm_capture(result,current,data['binding'],data['now_ns'])
    assert not result['accepted']
    assert result['completion_target_agreement']['translation_m']==pytest.approx(.0004)
    assert result['target_reference_pose'][0]==0.
    assert result['measured']['target_pose'][0]==.0002


def test_target_admission_accepts_quaternion_double_cover_at_entry_and_completion():
    from grasp_capture import confirm_capture
    qualify,data=case();data['measured']['target_pose'][3:]=[0.,0.,0.,-1.]
    result=qualify(**data)
    assert result['accepted'],result['reason']
    current=copy.deepcopy(data['measured']);current['target_pose'][3:]=[0.,0.,0.,1.]
    result=confirm_capture(result,current,data['binding'],data['now_ns'])
    assert result['accepted'],result['reason']
    assert result['measurement_target_agreement']['quaternion_distance']==0.
    assert result['completion_target_agreement']['quaternion_distance']==0.


@pytest.mark.parametrize('phase',['entry','completion'])
@pytest.mark.parametrize('change',['nan','infinite','nonunit','missing'])
def test_target_admission_rejects_malformed_measurements_at_entry_and_completion(phase,change):
    from grasp_capture import confirm_capture
    qualify,data=case();changed=copy.deepcopy(data['measured'])
    if change=='nan':changed['target_pose'][0]=float('nan')
    if change=='infinite':changed['target_pose'][1]=float('inf')
    if change=='nonunit':changed['target_pose'][6]=2.
    if change=='missing':changed.pop('target_pose')
    if phase=='entry':
        data['measured']=changed
        result=qualify(**data)
    else:
        result=confirm_capture(qualify(**data),changed,data['binding'],data['now_ns'])
        assert result['completion_measurement'] is changed
    assert not result['accepted']


def test_stl_scale_and_hash_are_collision_evidence(tmp_path):
    import struct
    from grasp_capture import CaptureModel
    mesh=tmp_path/'synthetic_mm.stl'
    mesh.write_bytes(b'synthetic'.ljust(80,b'\0')+struct.pack('<I',1)+struct.pack('<12fH',1,0,0,0,0,0,0,10,0,0,0,10,0))
    urdf=f'''<robot><link name="world"/><link name="a"><collision><geometry><mesh filename="{mesh}" scale="0.001 0.001 0.001"/></geometry></collision></link><link name="b"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link><joint name="a_fixed" type="fixed"><parent link="world"/><child link="a"/></joint><joint name="b_fixed" type="fixed"><parent link="world"/><child link="b"/></joint></robot>'''
    model=CaptureModel(urdf,['a','b'])
    assert model.facets['a'].max(axis=(0,1)).tolist()==pytest.approx([0,.01,.01])
    assert model.meshes[0]['scale']==[.001,.001,.001]
    assert len(model.meshes[0]['sha256'])==64


@pytest.mark.parametrize('config',[[],False,{'unknown':1},{'patch_radius_m':float('nan')}])
def test_malformed_configuration_never_silently_uses_defaults(config):
    qualify,data=case();data['config']=config
    assert not qualify(**data)['accepted']


@pytest.mark.parametrize('kind',['unilateral','incompatible_depth'])
def test_link_membership_cannot_replace_actual_common_contact_region(kind):
    from grasp_capture import CaptureModel
    import xml.etree.ElementTree as ET
    qualify,data=case();robot=copy.deepcopy(data['model'].robot)
    if kind=='unilateral':
        collision=robot.find("link[@name='right']/collision")
        collision.insert(0,ET.fromstring('<origin xyz="0.03 0 0"/>'))
    else:
        for name,z in [('left',.026),('right',-.026)]:
            robot.find(f"link[@name='{name}']/collision").insert(0,ET.fromstring(f'<origin xyz="0 0 {z}"/>'))
    data['model']=CaptureModel(ET.tostring(robot,encoding='unicode'),['left','right'])
    data['binding']['robot_model_sha256']=data['model'].sha256
    result=qualify(**data)
    assert not result['accepted']
    assert ('contact' if kind=='unilateral' else 'edge/depth mismatch') in result['reason']
