import copy
import importlib.util
from pathlib import Path

path=Path(__file__).resolve().parents[1]/'scripts/stage_a_rgbd/physics_owner/compare.py'
spec=importlib.util.spec_from_file_location('owner_compare',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def contact(a,b,positions):
    return dict(owner_collision_id=a,collision1=a,collision2=b,
        position_m=positions,normal=[],depth_m=[])


def record(step=100):
    return dict(complete=True,run_id='session',world='a0',frame_id='world',
        contact_request_step=step,step=step,stamp_ns=step*1000000,dt_ns=1000000,support_collision_id=7,
        support_collision_name='support',pairs=[dict(collision_id=23),dict(collision_id=27)],
        diagnostic_collision_ids=[7,23,27],diagnostic_requested_ids=[7,23,27],
        diagnostic_contacts=[contact(23,7,[[0.,0.,0.],[1.,0.,0.]]),
            contact(7,23,[[1.,0.,0.],[0.,0.,0.]]),contact(27,7,[[2.,0.,0.]]),contact(7,27,[[2.,0.,0.]])])


def compare(a,b):
    assert hasattr(module,'compare_contact_traces'),'matched-step fail-closed verifier missing'
    return module.compare_contact_traces(a,b,'session',[100])


def test_order_is_canonical_but_missing_depth_and_normal_still_blocks():
    a=record();b=copy.deepcopy(a);b['diagnostic_contacts'].reverse()
    for c in b['diagnostic_contacts']:c['position_m'].reverse()
    result=compare([a],[b])
    assert result['positions_counts_pairs_equal'] is True
    assert result['decision']=='BLOCKED_MISSING_NORMAL_OR_DEPTH'


def test_empty_contact_record_is_not_equivalence():
    a=record();a['diagnostic_contacts']=[]
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_MISSING_CONTACT_EVIDENCE'


def test_missing_step_is_rejected():
    assert compare([],[])['decision']=='BLOCKED_STEP_SCHEDULE'


def test_wrong_timestamp_is_rejected():
    a=record();b=copy.deepcopy(a);b['stamp_ns']+=1
    assert compare([a],[b])['decision']=='BLOCKED_STEP_IDENTITY'


def test_missing_requested_collision_is_rejected():
    a=record();a['diagnostic_requested_ids']=[7,23]
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_CONTACT_REQUEST_INVENTORY'


def test_position_difference_has_no_invented_tolerance():
    a=record();b=copy.deepcopy(a);b['diagnostic_contacts'][0]['position_m'][0][0]=1e-15
    assert compare([a],[b])['decision']=='BLOCKED_CONTACT_DISCREPANCY'


def test_missing_support_pair_is_rejected():
    a=record();a['diagnostic_contacts']=a['diagnostic_contacts'][:2]
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_REQUIRED_PAIR_MISSING'


def test_duplicate_step_is_rejected():
    a=record()
    assert compare([a,a],[copy.deepcopy(a)])['decision']=='BLOCKED_STEP_SCHEDULE'


def test_wrong_session_is_rejected():
    a=record();a['run_id']='another-session'
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_STEP_IDENTITY'


def test_partial_depth_array_rejects_incomplete_evidence():
    a=record();a['diagnostic_contacts'][0]['depth_m']=[.00001]
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_INCOMPLETE_CONTACT_FIELDS'


def test_nonfinite_contact_is_rejected():
    a=record();a['diagnostic_contacts'][0]['position_m'][0][0]=float('nan')
    assert compare([a],[copy.deepcopy(a)])['decision']=='BLOCKED_INCOMPLETE_CONTACT_FIELDS'


def test_full_fields_compare_exactly_without_engine_equivalence_claim():
    a=record()
    for c in a['diagnostic_contacts']:
        c['normal']=[[0.,0.,1.] for _ in c['position_m']]
        c['depth_m']=[1e-8 for _ in c['position_m']]
    b=copy.deepcopy(a)
    for c in b['diagnostic_contacts']:
        c['position_m'].reverse();c['normal'].reverse();c['depth_m'].reverse()
    assert compare([a],[b])['decision']=='PASS_FINITE_CONTACT_COMPARISON'
    b['diagnostic_contacts'][0]['normal'][0][0]=1e-15
    assert compare([a],[b])['decision']=='BLOCKED_CONTACT_DISCREPANCY'


def test_different_run_collision_inventories_are_rejected():
    a=record();b=copy.deepcopy(a)
    b['diagnostic_collision_ids'].append(99);b['diagnostic_requested_ids'].append(99)
    assert compare([a],[b])['decision']=='BLOCKED_COLLISION_INVENTORY_MISMATCH'


def reference_and_owner():
    ref=record()
    for c in ref['diagnostic_contacts']:
        c['normal']=[[0.,0.,1. if c['collision1']!=7 else -1.] for _ in c['position_m']]
        c['depth_m']=[1e-8 for _ in c['position_m']]
    own=dict(complete=True,session='session',world='a0',frame_id='world',step=100,
        stamp_ns=100000000,dt_ns=1000000,dart_frames=100,
        shapes=[dict(collision_id=i,physics_shape_id=i+100,shape_node_identity=str(i),mobile=i!=7) for i in [7,23,27]],contacts=[])
    for c in ref['diagnostic_contacts']:
        if c['collision1']==7:continue
        for p,n,d in zip(c['position_m'],c['normal'],c['depth_m']):
            own['contacts'].append(dict(collision1=c['collision1'],collision2=7,position_m=p,normal=n,depth_m=d))
    return ref,own


def test_reference_and_direct_owner_fields_match_after_normal_orientation():
    ref,own=reference_and_owner()
    assert hasattr(module,'compare_reference_owner'),'reference-owner verifier missing'
    assert module.compare_reference_owner([ref],[own],'session',[100])['decision']=='PASS_FINITE_REFERENCE_OWNER'


def test_reference_owner_depth_difference_fails_without_tolerance():
    ref,own=reference_and_owner();own['contacts'][0]['depth_m']+=1e-20
    assert hasattr(module,'compare_reference_owner'),'reference-owner verifier missing'
    assert module.compare_reference_owner([ref],[own],'session',[100])['decision']=='BLOCKED_REFERENCE_OWNER_CONTACT_FIELDS'


def test_reference_owner_mapping_mismatch_rejects():
    ref,own=reference_and_owner();own['shapes'][0]['collision_id']=99
    assert hasattr(module,'compare_reference_owner'),'reference-owner verifier missing'
    assert module.compare_reference_owner([ref],[own],'session',[100])['decision']=='BLOCKED_OWNER_INVENTORY'
