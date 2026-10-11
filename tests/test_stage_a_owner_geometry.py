"""Synthetic owner readback tests; these do not qualify a collision detector."""
import copy
from fractions import Fraction as F
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_physics_contact as m


def fixture():
    def shape(i,mobile,z,dims):
        t=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,z],[0.,0.,0.,1.]]
        return dict(collision_id=i,physics_shape_id=i+100,shape_node_identity=str(i),
            collision_name='a0::'+str(i),mobile=mobile,shape_type='BoxShape',dimensions_m=dims,
            dimensions_hex=[v.hex() for v in dims],world_transform=t,
            world_transform_hex=[[v.hex() for v in row] for row in t],
            pre_step_world_transform=copy.deepcopy(t),
            pre_step_world_transform_hex=[[v.hex() for v in row] for row in t])
    o=dict(complete=True,session='s',world='a0',frame_id='world',step=100,stamp_ns=100000000,
        dt_ns=1000000,dart_frames=100,pre_step_dart_frames=99,
        geometry_state_phase='post_ForwardStep_position_integration',
        shapes=[shape(7,False,-1050.,[2100.]*3),shape(23,True,.01249999,[.025]*3)],
        contacts=[dict(collision1=23,collision2=7,node1='23',node2='7',position_m=[0.,0.,0.],normal=[0.,0.,1.],depth_m=1e-8)])
    r=dict(complete=True,run_id='s',world='a0',frame_id='world',step=100,stamp_ns=100000000,dt_ns=1000000,
        support_collision_id=7,support_collision_name='a0::7',diagnostic_collision_ids=[7,23],
        diagnostic_requested_ids=[7,23],contact_request_step=100,
        pairs=[dict(collision_id=23,collision_name='a0::23',dimensions_m=[.025]*3)])
    return o,r


def evaluate(o,r):return m.enclose_owner_geometry(o,r,'s',100)


def set_entry(s,key,i,j,v):
    s[key][i][j]=v;s[key+'_hex'][i][j]=v.hex()


def test_exact_stored_geometry_enclosure_has_no_physical_authority():
    o,r=fixture();a=evaluate(o,r)
    assert a['stored_geometry_decision']=='PASS'
    bound=a['pairs'][0]['post_step']
    assert 0<bound['stored_dart_penetration_upper_m']<1e-7
    gap=F(.01249999)-F(.025)/2
    assert F(bound['minimum_gap_interval_m'][0])<=gap<=F(bound['minimum_gap_interval_m'][1])
    assert -gap<=F(bound['stored_dart_penetration_upper_m'])
    assert a['physical_penetration_upper_m'] is None and not a['contact_authority']
    assert a['decision']=='BLOCKED_COLLISION_DETECTOR_TRANSFORM_ERROR'


@pytest.mark.parametrize('case', ['deep','rotated','mapping','contacts','shape','stale','inventory','footprint','numeric','support','prestep','altered_support','frame','normal'])
def test_unsafe_or_incomplete_records_fail_closed(case):
    o,r=fixture();s=o['shapes'][1]
    if case=='deep':set_entry(s,'world_transform',2,3,.0123)
    if case=='rotated':
        # Exact 3-4-5 rotation: a lower corner, not the centre, violates 0.1 mm.
        for i,row in enumerate([[1.,0.,0.],[0.,.8,-.6],[0.,.6,.8]]):
            for j,v in enumerate(row):set_entry(s,'world_transform',i,j,v)
    if case=='mapping':o['contacts'][0]['node1']='7'
    if case=='contacts':o['contacts']=[]
    if case=='shape':s['shape_type']='SphereShape'
    if case=='stale':o['step']=99
    if case=='inventory':o['shapes'].pop(0)
    if case=='footprint':set_entry(s,'world_transform',0,3,1050.)
    if case=='numeric':s['dimensions_hex'][0]='0x1p-1'
    if case=='support':set_entry(o['shapes'][0],'world_transform',0,0,.9)
    if case=='prestep':o['pre_step_dart_frames']=97
    if case=='altered_support':
        o['shapes'][0]['dimensions_m']=[2099.]*3
        o['shapes'][0]['dimensions_hex']=[v.hex() for v in [2099.]*3]
    if case=='frame':o['frame_id']='camera'
    if case=='normal':o['contacts'][0].pop('normal')
    with pytest.raises(ValueError):evaluate(o,r)


def test_scalar_cannot_grant_numeric_authority():
    o,r=fixture();o['backend_error_bound_m']=0
    assert evaluate(o,r)['physical_penetration_upper_m'] is None
