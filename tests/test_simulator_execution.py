"""Measured execution guards: synthetic unit fixtures are never physical evidence."""
import sys, copy, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import pytest


def test_measured_attachment_supplies_actual_pose_without_duplicate_world_remove():
    from types import SimpleNamespace as N
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from simulator_execution import measured_attachment
    original=CollisionObject()
    original.id='runtime::part_06';original.header.frame_id='world'
    original.pose.orientation.w=1.
    original.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.025]*3)]
    original.primitive_poses=[Pose()]
    original.primitive_poses[0].orientation.w=1.
    measurements=N(frame=lambda s,f:[1.,2.,3.,0.,0.,0.,1.],
                   object_pose=lambda s,n:[1.01,2.02,3.03,0.,0.,0.,1.])
    diff=measured_attachment(original,dict(grasp_frame='tool',allowed_touch_links=['left','right']),measurements,{})
    # MoveIt processes attached ADD before world updates and consumes the
    # same-ID world object itself. A following REMOVE rejects the whole diff.
    assert diff.world.collision_objects==[]
    attached=diff.robot_state.attached_collision_objects[0]
    assert attached.object.id=='runtime::part_06'
    assert attached.link_name==attached.object.header.frame_id=='tool'
    assert attached.touch_links==['left','right']
    assert attached.object.primitives==original.primitives
    pose=attached.object.pose
    assert [pose.position.x,pose.position.y,pose.position.z]==pytest.approx([.01,.02,.03])
    assert pose.orientation.w==1.
    assert original.header.frame_id=='world' and original.pose.position.x==0.


def recorded_pile_guard(tmp_path, monkeypatch):
    """Replay the escaped five-neighbor state through real compiled FCL policy."""
    import xml.etree.ElementTree as ET
    from types import SimpleNamespace as N
    from simulator_execution import ContactGuard
    data=json.loads((Path(__file__).parent/'fixtures/stage_a1_pile_contacts.json').read_text())
    sample=data['sample']
    monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.001)
    monkeypatch.setattr('simulator_execution.time.time_ns',lambda:sample['wall_ns']+1000000)
    models=''.join(f'<model name="{o["object_id"]}"><link name="link"><collision name="collision"><geometry><box><size>0.025 0.025 0.025</size></box></geometry></collision></link></model>' for o in data['objects'])
    (tmp_path/'world.sdf').write_text('<sdf><world name="a0">'+models+'</world></sdf>')
    m=N(receipt=dict(world='a0',model='workcell_robot',run_id=sample['run_id'],pid=sample['pid']),
        receipt_path=tmp_path/'receipt.json',robot=ET.fromstring('<robot/>'),
        fresh=lambda:sample,object_pose=lambda s,n:s['poses']['a0::'+n],
        frame=lambda s,n:[0,0,0,0,0,0,1],joints=lambda s:{},record=lambda p,s:None)
    guard=ContactGuard(m,'runtime::part_06',[.025]*3,['left','right'],None,
                       N(entry_names=[],entry_values=[]),'tool')
    guard.bind_pile(data['objects'],data['binding'])
    return guard,sample,data


def provisional_pile_guard(tmp_path,monkeypatch):
    import threading
    import xml.etree.ElementTree as ET
    guard,sample,data=recorded_pile_guard(tmp_path,monkeypatch)
    sample['poses']['a0::part_06']=[0,0,0,0,0,0,1]
    sample['poses']['a0::part_00']=[-.025,0,0,0,0,0,1]
    sample['poses']['a0::part_07']=[0,.025,0,0,0,0,1]
    sample['contacts']=[]
    guard.phase='closing';guard.arm_names=set();guard.open_position=0.
    guard.m.lock=threading.RLock();guard.m.drain=lambda:[]
    guard.m.robot=ET.fromstring('<robot><ros2_control><joint name="leader"><command_interface name="position"/></joint></ros2_control></robot>')
    guard.m.joints=lambda s:{'leader':[.3,0.]}
    for side in ['left','right']:
        name=f'a0::workcell_robot::{side}::collision'
        sample['collisions'].append(name)
        sample['contacts'].append(dict(a='a0::part_06::link::collision',b=name,points=[[0,0,0]]))
    def contact(name,point):
        return dict(a='a0::part_06::link::collision',b=f'a0::part_{name}::link::collision',points=[point])
    def advance():
        sample['iteration']+=1;sample['sim_ns']+=1000000;sample['wall_ns']+=1000000
    sample['contacts'].append(contact('07',[0,.0125,0]))
    return guard,sample,advance,contact


def test_live_postclose_admission_carries_current_geometry_then_freezes(tmp_path,monkeypatch):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission()
    assert guard.pile_certificate['certified_set']==['runtime::part_07']
    advance();sample['contacts'].append(contact('00',[-.0125,0,0]));guard.check(sample)
    admitted_iteration=sample['iteration']
    # No repeated physical point is required while every fresh geometry query
    # still proves numerical contact. The neighbor itself does not move.
    sample['contacts']=sample['contacts'][:2]
    sample['poses']['a0::part_06'][1]=-.0002
    sample['poses']['a0::part_06'][2]=.00001
    for _ in range(300):advance();guard.check(sample)
    assert guard.pile_expired=={'runtime::part_07'}
    assert guard.pile_certificate['active_set']==['runtime::part_00']
    assert guard.pile_certificate['admissions'][1]['iteration']==admitted_iteration
    relative=guard.establish()
    assert len(relative)==7
    assert guard.pile_certificate['frozen'] is True
    assert guard.pile_certificate['certified_set']==['runtime::part_00']
    assert guard.held_start_sim_ns==sample['sim_ns']
    assert guard.pile_certificate['freeze']['iteration']==sample['iteration']
    assert guard.pile_certificate['freeze']['freshness_ms']==pytest.approx(1.)
    assert guard.pile_certificate['initial_bottom_m']==pytest.approx(-.01249)
    assert guard.pile_certificate['admissions'][0]['target_pose'][2]==0.
    assert guard.pile_certificate['rejected']==[]
    # Once frozen, even an exact, shallow newly measured pair cannot be added.
    advance();sample['poses']['a0::part_01']=[.025,-.0002,0,0,0,0,1]
    sample['contacts'].append(contact('01',[.0125,-.0002,0]))
    with pytest.raises(RuntimeError,match='new uncertified pile contact.*part_01'):guard.check(sample)
    failure=guard.pile_certificate['rejected_sample']
    assert failure['iteration']==sample['iteration']
    assert failure['contacts'][-1]['b']=='a0::part_01::link::collision'


@pytest.mark.parametrize('freeze',[False,True])
def test_provisional_expiry_is_irreversible_before_and_after_freeze(tmp_path,monkeypatch,freeze):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission()
    advance();sample['contacts']=sample['contacts'][:2]
    sample['poses']['a0::part_06'][1]=-.0002;guard.check(sample)
    assert guard.pile_expired=={'runtime::part_07'}
    if freeze:guard.establish()
    advance();sample['poses']['a0::part_06'][1]=0.
    sample['contacts'].append(contact('07',[0,.0125,0]))
    with pytest.raises(RuntimeError,match='recontact.*part_07'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


@pytest.mark.parametrize('bad',['stale','gap','deep','neighbor_moved'])
def test_provisional_admission_preserves_freshness_sequence_and_geometry(tmp_path,monkeypatch,bad):
    guard,sample,advance,contact=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();advance()
    if bad=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.251)
    if bad=='gap':sample['iteration']+=1
    if bad=='deep':sample['poses']['a0::part_06'][1]+=.001
    if bad=='neighbor_moved':sample['poses']['a0::part_07'][0]+=.001
    with pytest.raises(RuntimeError):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


def test_frozen_live_certificate_rejects_sequence_gap_and_preserves_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();advance();guard.check(sample);guard.establish()
    advance();sample['iteration']+=1
    with pytest.raises(RuntimeError,match='skipped measurement'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']


def test_live_certificate_preserves_rejected_opposing_contact_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish();advance()
    sample['contacts']=sample['contacts'][1:]
    with pytest.raises(RuntimeError,match='retention lost'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['contacts']==sample['contacts']


def test_checked_current_drains_pending_before_latest_retention_sample(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    pending=[]
    for _ in range(4):advance();pending.append(copy.deepcopy(sample))
    def drain():
        result=pending[:];pending.clear();return result
    guard.m.drain=drain
    current=guard.checked_current()
    assert current['iteration']==sample['iteration']
    assert pending==[]
    assert guard.pile_last_iteration==sample['iteration']
    # Retention uses that same serialized helper, never latest-before-queue.
    for _ in range(4):advance();pending.append(copy.deepcopy(sample))
    evidence=guard.retention_evidence(min_duration_ns=0)
    assert evidence['final_iteration']==sample['iteration']
    assert pending==[]
    assert guard.pile_certificate['rejected']==[]


def test_recorded_five_neighbor_certificate_uses_measured_geometry(tmp_path,monkeypatch):
    guard,sample,data=recorded_pile_guard(tmp_path,monkeypatch)
    evidence=guard.certify_pile(sample)
    assert evidence['certified_set']==['runtime::part_00','runtime::part_01','runtime::part_03','runtime::part_04','runtime::part_07']
    assert evidence['binding']==data['binding']
    assert evidence['iteration']==41679
    assert all(0<=c['geometry']['depth_m']<=.0001 for c in evidence['contacts'])
    assert guard.identity('a0::part_00::link::collision')=='runtime::part_00'
    with pytest.raises(RuntimeError,match='identity'):
        guard.identity('a0::part_00::other_link::collision')


def test_pile_certificate_rejects_unknown_identity_stale_or_deep_contact(tmp_path,monkeypatch):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    for change in ('unknown','stale','deep','point','preclose'):
        bad=copy.deepcopy(sample)
        if change=='unknown':bad['contacts'][0]['a']='a0::intruder::link::collision'
        if change=='stale':bad['wall_ns']-=300000000
        if change=='deep':bad['poses']['a0::part_06'][2]-=.001
        if change=='point':bad['contacts'][0]['points'][0][0]+=.01
        if change=='preclose':guard.pile_binding['close_goal_terminal_wall_ns']=sample['wall_ns']+1
        with pytest.raises(RuntimeError):guard.certify_pile(bad)
        assert guard.pile_certificate['rejected']


@pytest.mark.parametrize('phase',['closing','lift'])
def test_pile_allowance_expires_and_cannot_recapture_contact(tmp_path,monkeypatch,phase):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    guard.phase=phase
    if phase=='lift':guard.begin_pile_separation(sample)
    separated=copy.deepcopy(sample);separated['iteration']+=1;separated['sim_ns']+=1000000
    separated['poses']['a0::part_06'][2]+=.001;separated['contacts']=[]
    guard.check_pile(separated)
    # The four lower supports clear after 1 mm. The same-height side neighbor
    # remains touching in measured FCL geometry and MUST NOT expire yet.
    assert guard.pile_expired=={'runtime::part_00','runtime::part_01','runtime::part_03','runtime::part_04'}
    with pytest.raises(RuntimeError,match='recontact'):guard.check_pile(sample)


@pytest.mark.parametrize('delta',[(.003,0,0),(0,0,-.00001),(0,0,.011)])
def test_pile_departure_preserves_bounded_monotonic_corridor(tmp_path,monkeypatch,delta):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample);guard.phase='lift';guard.begin_pile_separation(sample)
    moved=copy.deepcopy(sample)
    moved['poses']['a0::part_06'][:3]=[a+b for a,b in zip(moved['poses']['a0::part_06'][:3],delta)]
    with pytest.raises(RuntimeError):guard.check_pile(moved)


def test_pile_certificate_rechecks_current_depth_and_neighbor_motion(tmp_path,monkeypatch):
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    bad=copy.deepcopy(sample);bad['poses']['a0::part_00'][2]+=.001
    with pytest.raises(RuntimeError):guard.check_pile(bad)


def test_measured_fcl_pile_exception_requires_exact_active_pair_and_depth(tmp_path,monkeypatch):
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    guard,sample,_=recorded_pile_guard(tmp_path,monkeypatch)
    guard.certify_pile(sample)
    def contact(neighbor='runtime::part_00',depth=.00001,kind=2):
        return N(contact_body_1=guard.object,contact_body_2=neighbor,body_type_1=kind,body_type_2=1,
                 position=N(x=0.,y=0.,z=.025),normal=N(x=0.,y=0.,z=-1.),depth=depth)
    validate_measured_contacts(N(valid=False,contacts=[contact()]),None,False,pile_guard=guard)
    for c in (contact('runtime::intruder'),contact(depth=.000101),contact(kind=0)):
        with pytest.raises(RuntimeError):
            validate_measured_contacts(N(valid=False,contacts=[c]),None,False,pile_guard=guard)
    guard.pile_expired.add('runtime::part_00')
    with pytest.raises(RuntimeError):
        validate_measured_contacts(N(valid=False,contacts=[contact()]),None,False,pile_guard=guard)


def test_measurement_rejects_stale_wrong_run_and_missing_samples():
    from simulator_execution import validate_sample
    sample=dict(run_id='r',pid=12,iteration=10,sim_ns=10000000,wall_ns=1000000000,
                poses={'cell::part':[0,0,0,0,0,0,1]},joints={'cell::leader':[.3,0]},contacts=[],collisions=['cell::part::body::shape'])
    receipt=dict(run_id='r',pid=12)
    validate_sample(sample,receipt,1.01,9)
    for patch in [dict(run_id='wrong'),dict(pid=13),dict(iteration=8),dict(wall_ns=0),dict(poses={}),dict(error='broken')]:
        with pytest.raises(RuntimeError):validate_sample(dict(sample,**patch),receipt,1.01,9)


def test_support_expiry_and_non_monotonic_separation():
    from simulator_execution import Separation
    p=Separation([0,0,.01,0,0,0,1],0)
    p.check([0,0,.01001,0,0,0,1],0,True)
    p.check([0,0,.011,0,0,0,1],.001,False)
    assert p.expired
    with pytest.raises(RuntimeError):p.check([0,0,.011,0,0,0,1],.001,True)
    for pose in ([.003,0,.01,0,0,0,1],[0,0,.009,0,0,0,1]):
        with pytest.raises(RuntimeError):Separation([0,0,.01,0,0,0,1],0).check(pose,0,True)


def test_grasp_requires_two_contacting_fingers_and_rejects_slip():
    from simulator_execution import HeldObject
    pose=[0,0,0,0,0,0,1]
    for fingers in ([],['left']):
        with pytest.raises(RuntimeError):HeldObject(pose,pose,fingers,['left','right'],.3,0)
    held=HeldObject(pose,pose,['left','right'],['left','right'],.3,0)
    held.check([0,0,.1,0,0,0,1],[0,0,.1,0,0,0,1],['left','right'])
    with pytest.raises(RuntimeError):held.check([0,0,.1,0,0,0,1],[.004,0,.1,0,0,0,1],['left','right'])
    with pytest.raises(RuntimeError):held.check(pose,pose,['left'])


def test_cancel_waits_for_terminal_and_measured_stop_then_reconciles():
    from simulator_execution import cancel_owned
    events=[]
    cancel_owned(lambda:events.append('cancel') or True,lambda:events.append('terminal') or True,
                 lambda:events.append('stop') or True,lambda:events.append('revoke'),lambda:events.append('reconcile'))
    assert events==['cancel','terminal','stop','revoke','reconcile']
    events=[]
    with pytest.raises(RuntimeError):
        cancel_owned(lambda:True,lambda:True,lambda:False,lambda:events.append('revoke'),lambda:events.append('reconcile'))
    assert events==['revoke']


def test_runtime_uses_same_compiled_floor_predicate():
    import ctypes
    from ament_index_python.packages import get_package_prefix
    lib=ctypes.CDLL(str(Path(get_package_prefix('workcell_builder'))/'lib/libworkcell_support_contact.so'))
    f=lib.workcell_support_contact_valid
    f.argtypes=[ctypes.c_char_p]*4+[ctypes.c_double,ctypes.POINTER(ctypes.c_double)];f.restype=ctypes.c_bool
    def valid(a='floor',b='part',p=(0,0,0,0,0,1,.00001)):
        return f(b'part',b'floor',a.encode(),b.encode(),0,(ctypes.c_double*7)(*p))
    assert valid()
    assert not valid(a='wrong_support')
    assert not valid(b='unrelated')
    assert not valid(p=(0,0,0,1,0,0,.00001))
    assert not valid(p=(0,0,.1,0,0,1,.00001))
    assert not valid(p=(0,0,0,0,0,1,.001))


def test_failed_closure_and_cancel_do_not_authorize_full_cycle(tmp_path):
    from simulator_execution import require_trial_evidence
    p=tmp_path/'trials.json';p.write_text('[{"result":"FAIL"}]')
    with pytest.raises(RuntimeError):require_trial_evidence(p,{})


def test_live_collision_query_keeps_carried_collisions_and_expires_support():
    from simulator_execution import validate_measured_contacts
    from types import SimpleNamespace as N
    def contact(a='support',b='part',z=0,nz=1,depth=.00001):
        return N(contact_body_1=a,contact_body_2=b,body_type_1=1,body_type_2=2,position=N(x=0,y=0,z=z),normal=N(x=0,y=0,z=nz),depth=depth)
    response=N(valid=False,contacts=[contact()])
    policy=dict(object_id='part',support_id='support',floor_z=0)
    validate_measured_contacts(response,policy,False)
    for contacts in ([contact(a='wall')],[contact(z=.1)],[contact(nz=0)],[contact(b='other')],[]):
        with pytest.raises(RuntimeError):validate_measured_contacts(N(valid=False,contacts=contacts),policy,False)
    with pytest.raises(RuntimeError):validate_measured_contacts(response,policy,True)


def test_bridge_publisher_identity_is_node_not_executable_name():
    from simulator_execution import validate_publisher
    from types import SimpleNamespace as N
    validate_publisher([N(node_name='ros_gz_bridge',node_namespace='/')])
    for pubs in ([],[N(node_name='parameter_bridge',node_namespace='/')],
                 [N(node_name='ros_gz_bridge',node_namespace='/')]*2):
        with pytest.raises(RuntimeError):validate_publisher(pubs)


def test_controlled_cancel_requires_measured_arm_movement_after_goal_acceptance():
    from simulator_execution import CancellationMotion
    start=dict(iteration=10,sim_ns=100000000,wall_ns=1000000000)
    trial=CancellationMotion('owned',start,{'arm':[0.,0.]},{'arm'},accepted_wall_ns=1100000000)
    assert not trial.observe(dict(start,iteration=11,sim_ns=200000000),{'arm':[.003,.01]})
    assert not trial.observe(dict(start,iteration=12,sim_ns=210000000,wall_ns=1110000000),{'arm':[.002,0.]})
    assert trial.observe(dict(start,iteration=13,sim_ns=220000000,wall_ns=1120000000),{'arm':[.003,.01]})
    assert trial.evidence['movement']['joints']['arm']==[.003,.01]
    assert trial.evidence['movement']['max_displacement_rad']==pytest.approx(.003)
    with pytest.raises(RuntimeError,match='missing'):
        CancellationMotion('owned',start,{'gripper':[0.,0.]},{'arm'})


def test_cancel_acknowledgement_must_name_the_owned_goal():
    from simulator_execution import cancel_response_matches
    from types import SimpleNamespace as N
    response=lambda ids:N(return_code=0,goals_canceling=[N(goal_id=N(uuid=i)) for i in ids])
    owned=[1]*16
    assert cancel_response_matches(owned,response([owned]))
    assert not cancel_response_matches(owned,response([[2]*16]))
    assert not cancel_response_matches(owned,response([]))
    assert not cancel_response_matches(owned,N(return_code=1,goals_canceling=response([owned]).goals_canceling))


def test_stop_window_requires_complete_contiguous_sustained_measured_stop():
    from simulator_execution import StopWindow
    window=StopWindow({'arm','gripper'})
    def sample(i,ns):return dict(iteration=i,sim_ns=ns,wall_ns=1000000000+ns)
    stopped={'arm':[.3,0.],'gripper':[0.,0.]}
    assert not window.observe(sample(1,0),stopped)
    assert not window.observe(sample(2,200000000),stopped)
    assert not window.observe(sample(3,300000000),dict(stopped,arm=[.3,.01]))
    assert not window.observe(sample(4,400000000),stopped)
    assert window.observe(sample(5,700000000),stopped)
    assert window.evidence['duration_sim_ns']==300000000
    assert len(window.evidence['samples'])==2
    with pytest.raises(RuntimeError,match='missing'):StopWindow({'arm'}).observe(sample(1,0),{})
    with pytest.raises(RuntimeError,match='gap'):window.observe(sample(7,800000000),stopped)


def test_cancellation_acceptance_requires_movement_canceled_stop_and_alive_cleanup():
    from simulator_execution import require_cancellation_acceptance
    accepted=dict(cancellation_confirmed=True,cancellation_accepted=True,cancellation_movement_verified=True,
        interrupted_action_terminal_status=5,motion_stop_verified=True,
        controller_cancellation={'arm':dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)},
        owned_execution_goal=dict(uuid='01',accepted=True),
        cancellation_motion=dict(movement={'iteration':12}),
        stopped_window=dict(duration_sim_ns=300000000,samples=[{},{}]),
        measured_reconciliation=dict(acm_restored=True,measured_geometry_matches=True,held=False,attached_ids=[]),
        recovery_scene=dict(contact_acm_restored=True,attached_ids=[]))
    require_cancellation_acceptance(accepted)
    for patch in (dict(cancellation_movement_verified=False),dict(interrupted_action_terminal_status=4),
                  dict(cancellation_accepted=False),dict(motion_stop_verified=False),
                  dict(recovery_inspection_failure='service unavailable'),
                  dict(measured_reconciliation=dict(acm_restored=True,measured_geometry_matches=False,held=False,attached_ids=[])),
                  dict(recovery_scene=dict(contact_acm_restored=False,attached_ids=[]))):
        with pytest.raises(RuntimeError):require_cancellation_acceptance(dict(accepted,**patch))


def test_early_successful_approach_cannot_fall_through_cancel_trial():
    """Exercise the actual nested action owner with an immediately ended action."""
    import ast,time
    from types import SimpleNamespace as N
    tree=ast.parse((Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    owner=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    def future(result):return N(done=lambda:True,result=lambda:result)
    terminal=future(N(status=4,result=N(error_code=N(val=1))))
    handle=N(accepted=True,goal_id=N(uuid=[1]*16),get_result_async=lambda:terminal,
             cancel_goal_async=lambda:future(N(return_code=3,goals_canceling=[])))
    client=N(wait_for_server=lambda **kw:True,send_goal_async=lambda goal:future(handle))
    sample=dict(iteration=1,sim_ns=100,wall_ns=time.time_ns())
    summary={};events=[]
    context=dict(time=time,node=None,summary=summary,measurements=N(fresh=lambda:sample,joints=lambda s:{'arm':[0.,0.]}),
        execute_client=client,controlled_cancel=True,controller_audit=None,contract={'home_joint_names':['arm']},
        rclpy=N(spin_until_future_complete=lambda *args,**kw:None),execution_monitor=lambda:None,
        wait_stopped=lambda:True,apply=lambda diff:events.append('revoke'),PlanningScene=N,baseline=N(),
        measured_reconcile=lambda:events.append('reconcile'))
    exec(compile(ast.Module(body=[owner],type_ignores=[]),'<actual-action-owner>','exec'),context)
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory
    goal=ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'])))
    with pytest.raises(RuntimeError,match='ENDED_BEFORE_CANCEL'):context['action'](client,goal,1)
    assert summary['owned_execution_goal']['accepted']
    assert summary['interrupted_action_terminal_status']==4
    assert summary['cancellation_confirmed'] is False
    assert events==['revoke']


def test_cancellation_requires_independent_controller_terminal():
    from simulator_execution import validate_controller_cancellation
    controller=dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)
    validate_controller_cancellation({'arm':controller})
    for bad in ({},{'arm':dict(controller,status=4)},{'arm':dict(controller,executing_before_cancel=False)}):
        with pytest.raises(RuntimeError):validate_controller_cancellation(bad)


def test_acquisition_gap_is_latched_even_if_following_samples_are_valid():
    import threading,time,json
    from types import SimpleNamespace as N
    from simulator_execution import Measurements
    m=Measurements.__new__(Measurements);m.lock=threading.RLock();m.receipt=dict(run_id='r',pid=12)
    m.latest=None;m.pending=[];m.previous=1;m.armed=True;m.error=None;m.timing=[]
    def send(i):
        m.update(N(data=json.dumps(dict(run_id='r',pid=12,iteration=i,sim_ns=i,wall_ns=time.time_ns(),
            poses={'p':[0,0,0,0,0,0,1]},joints={'j':[0,0]},contacts=[],collisions=['c']))))
    send(3);send(2)
    with pytest.raises(RuntimeError,match='skipped'):m.fresh()


def test_cancel_metrics_include_motion_during_controller_result_wait():
    from simulator_execution import cancellation_metrics
    from types import SimpleNamespace as N
    anchor=dict(iteration=1,sim_ns=0,wall_ns=1000000000,joints={'arm':[0.,.1]})
    samples=[dict(iteration=i,sim_ns=(i-1)*100000000,wall_ns=1000000000+(i-1)*100000000,
                  joints={'arm':[.02,0.]}) for i in range(2,7)]
    summary=dict(cancellation_request={'wall_ns':1010000000},cancel_measurement=anchor,
        cancellation_motion={'accepted':{'joints':{'arm':[0.,.1]}}},
        stopped_window={'samples':[samples[2]]},
        controller_cancellation={'arm':{'result_wall_ns':1300000000,'terminal_status_receive_wall_ns':1250000000}})
    result=cancellation_metrics(summary,N(joints=lambda s:s['joints']),samples)
    assert result['cancel_to_measured_stop_ms']==90
    assert result['cancel_to_controller_result_ms']['arm']==240
    assert result['additional_joint_travel_rad']['arm']==pytest.approx(.02)


def test_motion_telemetry_metrics_keep_250ms_guard_and_contiguous_iterations():
    from simulator_execution import telemetry_metrics
    timing=[]
    for i in range(1,6):
        source=1_000_000_000+i*10_000_000
        timing.append(dict(event='receive',iteration=i,sim_ns=i*10_000_000,
            source_wall_ns=source,publish_wall_ns=source+1_000_000,
            source_serialization_ns=500_000,receive_wall_ns=source+2_000_000,
            callback_ns=200_000,error=None))
        timing.append(dict(event='fresh',iteration=i,read_wall_ns=source+3_000_000,
            age_ns=3_000_000,lock_wait_ns=10_000,error=None))
    result=telemetry_metrics(timing)
    assert result['samples']==5
    assert result['max_fresh_age_ms']==pytest.approx(3.)
    assert result['max_delivery_ms']==pytest.approx(2.)
    assert result['real_time_factor']==pytest.approx(1.)
    stale=copy.deepcopy(timing)
    stale[-1]['age_ns']=250_000_000
    with pytest.raises(RuntimeError,match='250 ms'):telemetry_metrics(stale)
    gap=copy.deepcopy(timing)
    gap[4]['iteration']=4
    with pytest.raises(RuntimeError,match='iteration gap'):telemetry_metrics(gap)


def test_full_cycle_prerequisites_require_all_four_qualified_trials(tmp_path):
    from simulator_execution import require_trial_evidence
    capability_sha='freshly-qualified-build'
    overlay_sha='freshly-qualified-tem-overlay'
    move_group_sha='freshly-qualified-move-group'
    current=dict(sha256=capability_sha,moveit_overlay={'library':{'sha256':overlay_sha},
        'move_group':{'executable':{'sha256':move_group_sha}}})
    common=dict(commissioning_capability=current,
        backend_identity={'backend':'simulator'},
        motion_backend_identity={'backend':'simulator'},
        measured_reconciliation={'acm_restored':True,'attached_ids':[],'measured_geometry_matches':True,'held':False},
        recovery_scene={'contact_acm_restored':True,'attached_ids':[]})
    cancellation=dict(common,result='CANCELLATION_TRIAL_PASS',
        cancellation_confirmed=True,cancellation_accepted=True,cancellation_movement_verified=True,
        interrupted_action_terminal_status=5,motion_stop_verified=True,
        controller_cancellation={'arm':dict(uuid='abc',executing_before_cancel=True,status=5,error_code=0,result_wall_ns=12)},
        owned_execution_goal=dict(uuid='01',accepted=True),
        cancellation_motion=dict(movement={'iteration':12}),
        stopped_window=dict(duration_sim_ns=300000000,samples=[{},{}]))
    telemetry=dict(common,result='MOTION_TELEMETRY_PASS',
        motion_telemetry={'max_fresh_age_ms':12.,'max_delivery_ms':8.})
    retention=dict(common,result='STATIONARY_RETENTION_PASS',
        stationary_retention={'duration_sim_ns':1_100_000_000,'samples':50})
    contact=dict(common,result='CONTACT_RELEASE_PASS',verified_lift_clearance_m=.02,
        release_evidence={'settled':True})
    records=[copy.deepcopy(r) for r in (cancellation,telemetry,retention,contact)]
    path=tmp_path/'evidence.json';path.write_text(json.dumps(records))
    accepted=require_trial_evidence(path,current)
    assert accepted['capability_sha256']==capability_sha
    assert accepted['moveit_overlay_sha256']==overlay_sha
    assert accepted['move_group_sha256']==move_group_sha
    for index in range(4):
        for invalid_overlay in ({'library':{'sha256':'different-tem-overlay'}},
                {'library':{'sha256':''}}, {'library':{}}, {}):
            invalid=copy.deepcopy(records)
            invalid[index]['commissioning_capability']['moveit_overlay']=invalid_overlay
            path.write_text(json.dumps(invalid))
            with pytest.raises(RuntimeError,match='MoveIt overlay'):
                require_trial_evidence(path,current)
        invalid=copy.deepcopy(records)
        del invalid[index]['commissioning_capability']['moveit_overlay']
        path.write_text(json.dumps(invalid))
        with pytest.raises(RuntimeError,match='MoveIt overlay'):
            require_trial_evidence(path,current)
    path.write_text(json.dumps(records))
    for invalid_current in ({'sha256':capability_sha},
            dict(current,moveit_overlay={'library':{'sha256':'stale-tem-overlay'}}),
            dict(current,moveit_overlay={'library':{'sha256':''}})):
        with pytest.raises(RuntimeError,match='MoveIt overlay'):
            require_trial_evidence(path,invalid_current)
    for index in range(4):
        for invalid_executable in ({'executable':{'sha256':'different-move-group'}},
                {'executable':{'sha256':''}}, {'executable':{}}, {}):
            invalid=copy.deepcopy(records)
            invalid[index]['commissioning_capability']['moveit_overlay']['move_group']=invalid_executable
            path.write_text(json.dumps(invalid))
            with pytest.raises(RuntimeError,match='MoveGroup executable'):
                require_trial_evidence(path,current)
        invalid=copy.deepcopy(records)
        del invalid[index]['commissioning_capability']['moveit_overlay']['move_group']
        path.write_text(json.dumps(invalid))
        with pytest.raises(RuntimeError,match='MoveGroup executable'):
            require_trial_evidence(path,current)
    path.write_text(json.dumps(records))
    for invalid_executable in ({'executable':{'sha256':'stale-move-group'}},
            {'executable':{'sha256':''}}, {'executable':{}}, {}):
        invalid_current=copy.deepcopy(current)
        invalid_current['moveit_overlay']['move_group']=invalid_executable
        with pytest.raises(RuntimeError,match='MoveGroup executable'):
            require_trial_evidence(path,invalid_current)
    invalid_current=copy.deepcopy(current)
    del invalid_current['moveit_overlay']['move_group']
    with pytest.raises(RuntimeError,match='MoveGroup executable'):
        require_trial_evidence(path,invalid_current)
    path.write_text(json.dumps([cancellation,telemetry,retention]))
    with pytest.raises(RuntimeError):require_trial_evidence(path,current)


def test_same_attempt_retention_proof_is_bound_to_actual_frozen_interval(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    start=copy.deepcopy(sample);frozen=copy.deepcopy(guard.pile_certificate['freeze'])
    for _ in range(1000):advance();guard.check(sample)
    proof=guard.retention_evidence()
    assert proof['run_id']==sample['run_id']
    assert proof['target']==guard.object
    assert proof['binding']==guard.pile_binding
    assert proof['binding']['close_goal_uuid']
    assert proof['binding']['resolution_sha256']
    assert proof['start_iteration']==start['iteration']
    assert proof['start_sim_ns']==start['sim_ns']
    assert proof['start_wall_ns']==start['wall_ns']
    assert proof['end_iteration']==proof['final_iteration']==sample['iteration']
    assert proof['end_sim_ns']==proof['final_sim_ns']==sample['sim_ns']
    assert proof['end_wall_ns']==sample['wall_ns']
    assert proof['duration_sim_ns']==1_000_000_000
    assert proof['required_contact_links']==proof['contact_links']==['left','right']
    assert guard.pile_certificate['freeze']==frozen
    assert guard.pile_certificate['certified_set']==['runtime::part_07']
    proof['binding']['close_goal_uuid']='foreign-close'
    assert guard.retention_evidence()['binding']['close_goal_uuid']==guard.pile_binding['close_goal_uuid']


@pytest.mark.parametrize('bad',['another_run','stale','changed_close','changed_resolution'])
def test_live_retention_rejects_foreign_stale_or_changed_attempt(tmp_path,monkeypatch,bad):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    for _ in range(1000):advance();guard.check(sample)
    if bad=='another_run':sample['run_id']='foreign-run'
    elif bad=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.251)
    elif bad=='changed_close':guard.pile_binding['close_goal_uuid']='foreign-close'
    else:guard.pile_binding['resolution_sha256']='foreign-resolution'
    with pytest.raises(RuntimeError):guard.retention_evidence()


@pytest.mark.parametrize('bad',['contact_loss','translation_slip','rotation_slip'])
def test_failed_current_hold_cannot_later_produce_retention_proof(tmp_path,monkeypatch,bad):
    import math
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    # Isolate the held-object limits from pile geometry; exact fingertips remain required.
    sample['contacts']=sample['contacts'][:2]
    guard.begin_pile_admission();guard.establish()
    for _ in range(500):advance();guard.check(sample)
    good=copy.deepcopy(sample)
    advance()
    if bad=='contact_loss':sample['contacts']=sample['contacts'][:1]
    elif bad=='translation_slip':sample['poses']['a0::part_06'][0]=.0021
    else:sample['poses']['a0::part_06'][3:]=[0,0,math.sin(.011/2),math.cos(.011/2)]
    with pytest.raises(RuntimeError,match='retention lost'):guard.check(sample)
    assert guard.pile_certificate['rejected_sample']['iteration']==sample['iteration']
    sample['contacts']=good['contacts'];sample['poses']=good['poses']
    for _ in range(500):advance()
    with pytest.raises(RuntimeError,match='missing valid pile certificate'):guard.retention_evidence()


def test_current_grasp_retention_cannot_reuse_a_shorter_hold(tmp_path,monkeypatch):
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission();guard.establish()
    for _ in range(999):advance();guard.check(sample)
    with pytest.raises(RuntimeError,match='duration is too short'):guard.retention_evidence()


def physics_pose_sample():
    """Synthetic declared-source contract, not relabelled historical physics data."""
    method='physics_link_frame_data_at_offset'
    receipt=dict(run_id='physics-run',pid=12,world='a0',measurement_pose_source=method)
    sample=dict(run_id='physics-run',pid=12,iteration=10,sim_ns=10_000_000,wall_ns=1_000_000_000,
        poses={'a0::part':[0,0,0,0,0,0,1],'a0::part::link':[0,0,0,0,0,0,1]},
        joints={'a0::robot::j':[0,0]},collisions=['collision'],contacts=[],
        pose_source=dict(method=method,frame='world',query_iteration=10,query_sim_ns=10_000_000,
                         gazebo_version='6.17.0',read_only=True),
        pose_entities={'a0::part':dict(entity=2,link_entity=3,query_entity=5,kind='model'),
                       'a0::part::link':dict(entity=3,link_entity=3,query_entity=5,kind='link')})
    return sample,receipt


def test_authoritative_physics_pose_source_accepts_complete_current_identity():
    from simulator_execution import validate_sample
    sample,receipt=physics_pose_sample()
    validate_sample(sample,receipt,1.01,9)


@pytest.mark.parametrize('bad',['missing_source','wrong_method','wrong_receipt_method','wrong_frame',
    'writable','missing_version','stale_iteration','stale_sim_time','missing_entities','missing_link',
    'extra_entity','zero_entity','wrong_link_identity','wrong_kind','foreign_world','query_error','stale_wall',
    'wrong_model_link','wrong_model_query','duplicate_entity','shared_link_query','query_is_original'])
def test_authoritative_physics_pose_source_fails_closed(tmp_path,bad):
    from simulator_execution import validate_sample
    sample,receipt=physics_pose_sample()
    if bad=='missing_source':sample.pop('pose_source')
    elif bad=='wrong_method':sample['pose_source']['method']='cached_ecs'
    elif bad=='wrong_receipt_method':receipt['measurement_pose_source']='cached_ecs'
    elif bad=='wrong_frame':sample['pose_source']['frame']='model'
    elif bad=='writable':sample['pose_source']['read_only']=False
    elif bad=='missing_version':sample['pose_source']['gazebo_version']=''
    elif bad=='stale_iteration':sample['pose_source']['query_iteration']-=1
    elif bad=='stale_sim_time':sample['pose_source']['query_sim_ns']-=1
    elif bad=='missing_entities':sample.pop('pose_entities')
    elif bad=='missing_link':sample['pose_entities'].pop('a0::part::link')
    elif bad=='extra_entity':sample['pose_entities']['a0::extra']=sample['pose_entities']['a0::part']
    elif bad=='zero_entity':sample['pose_entities']['a0::part']['query_entity']=0
    elif bad=='wrong_link_identity':sample['pose_entities']['a0::part::link']['link_entity']=20
    elif bad=='wrong_kind':sample['pose_entities']['a0::part']['kind']='visual'
    elif bad=='wrong_model_link':sample['pose_entities']['a0::part']['link_entity']=20
    elif bad=='wrong_model_query':sample['pose_entities']['a0::part']['query_entity']=20
    elif bad=='duplicate_entity':sample['pose_entities']['a0::part']['entity']=3
    elif bad=='shared_link_query':
        sample['poses']['a0::part::other']=[0,0,0,0,0,0,1]
        sample['pose_entities']['a0::part::other']=dict(entity=6,link_entity=6,query_entity=5,kind='link')
    elif bad=='query_is_original':
        for entity in sample['pose_entities'].values():entity['query_entity']=2
    elif bad=='foreign_world':
        sample['poses']['other::part']=sample['poses'].pop('a0::part')
        sample['pose_entities']['other::part']=sample['pose_entities'].pop('a0::part')
    elif bad=='query_error':sample['error']='physics pose query unavailable'
    elif bad=='stale_wall':sample['wall_ns']=700_000_000
    with pytest.raises(RuntimeError):validate_sample(sample,receipt,1.01,9)


@pytest.mark.parametrize('method',[None,'cached_ecs'])
def test_measurements_rejects_old_pose_receipt_before_acquisition(tmp_path,monkeypatch,method):
    from simulator_execution import Measurements
    receipt={'world':'a0'}
    if method is not None:receipt['measurement_pose_source']=method
    monkeypatch.setattr('simulator_backend.verify_receipt_process',lambda _:receipt)
    with pytest.raises(RuntimeError,match='physics pose source'):
        Measurements(None,tmp_path/'receipt.json',tmp_path/'measurements.jsonl')
    assert not (tmp_path/'measurements.jsonl').exists()


def release_boundary_guard(tmp_path, monkeypatch):
    """A lifted, still held version of the existing physical pile fixture."""
    guard, sample, advance, contact = provisional_pile_guard(tmp_path, monkeypatch)
    guard.begin_pile_admission();guard.establish()
    guard.phase = 'opening'
    guard.pile_expired = set(guard.pile_certificate['certified_set'])
    guard.pile_certificate['active_set'] = []
    sample['contacts'] = sample['contacts'][:2]
    position = [.3]
    guard.m.joints = lambda s: {'leader': [position[0], 0.]}
    goal = dict(uuid='a'*32,accepted=True,stage='COMMISSION_RELEASE',
        wall_ns=sample['wall_ns']+1,run_id=sample['run_id'],target=guard.object,
        resolution_sha256=guard.pile_binding['resolution_sha256'],
        execution_attempt=guard.pile_binding['execution_attempt'],
        selected_grasp_index=guard.pile_binding.get('selected_grasp_index',0))
    guard.begin_release(goal, copy.deepcopy(sample))
    return guard, sample, advance, position, goal, contact


def test_release_requires_owned_success_physical_loss_and_detachment(tmp_path, monkeypatch):
    guard, sample, advance, position, goal, _ = release_boundary_guard(tmp_path, monkeypatch)
    assert guard.ownership == 'RELEASING'
    with pytest.raises(RuntimeError, match='physical release'):
        guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    advance();position[0]=.28;sample['poses']['a0::part_06'][0]+=.0001
    guard.check(sample)
    assert guard.release_candidate is None  # Exact fingertips still touch.
    sample['contacts']=[];advance();guard.check(sample)
    assert guard.release_candidate['iteration']==sample['iteration']
    assert guard.ownership=='RELEASING'
    with pytest.raises(RuntimeError, match='terminal'):
        guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    with pytest.raises(RuntimeError, match='owned'):
        guard.terminal_release('b'*32,4,1,sample['wall_ns'])
    guard.terminal_release(goal['uuid'],4,1,sample['wall_ns']+1)
    with pytest.raises(RuntimeError, match='attachment'):
        guard.finish_release(dict(held=False,attached_ids=[guard.object],acm_restored=True,measured_geometry_matches=True))
    assert guard.ownership=='RELEASING'
    position[0]=0.
    with pytest.raises(RuntimeError,match='post-terminal'):
        guard.release_ready()
    advance()
    evidence=guard.finish_release(dict(held=False,attached_ids=[],acm_restored=True,measured_geometry_matches=True))
    assert evidence['run_id']==sample['run_id'] and evidence['target']==guard.object
    assert evidence['goal']['uuid']==goal['uuid']
    assert evidence['separation']['wall_ns']<evidence['goal']['terminal_wall_ns']
    assert guard.ownership=='FREE_SETTLING' and guard.held is None
    with pytest.raises(RuntimeError, match='cannot regress'):
        guard.begin_release(goal,sample)
    sample['contacts']=[dict(a='a0::workcell_robot::arm_link::collision',
                             b='a0::part_06::link::collision',points=[[0,0,0]])]
    advance()
    with pytest.raises(RuntimeError,match='released target recontacted robot/tool'):guard.check(sample)


def test_release_landing_is_accepted_only_after_measured_loss_and_robot_contact_still_fails(tmp_path,monkeypatch):
    guard,sample,advance,position,goal,contact=release_boundary_guard(tmp_path,monkeypatch)
    advance();position[0]=.28;sample['poses']['a0::part_06'][0]+=.0001
    guard.check(sample)
    sample['contacts']=[];advance();guard.check(sample)
    assert guard.release_candidate is not None
    advance();sample['contacts']=[contact('07',[0,.0125,0])]
    guard.check(sample)  # Expired pair landing is no longer carried-object recontact.
    assert guard.ownership=='RELEASING'
    sample['contacts'].append(dict(a='a0::workcell_robot::arm_link::collision',
                                  b='a0::part_07::link::collision',points=[[0,0,0]]))
    advance()
    with pytest.raises(RuntimeError,match='unpermitted physical contact'):guard.check(sample)


@pytest.mark.parametrize('fault',['other_run','stale','foreign_goal','failed_open','no_motion'])
def test_release_rejects_foreign_stale_failed_or_unmoved_evidence(tmp_path,monkeypatch,fault):
    guard,sample,advance,position,goal,_=release_boundary_guard(tmp_path,monkeypatch)
    advance()
    if fault=='other_run':sample['run_id']='foreign'
    elif fault=='stale':monkeypatch.setattr('simulator_execution.time.time',lambda:sample['wall_ns']/1e9+.3)
    elif fault!='no_motion':position[0]=.28
    sample['contacts']=[];sample['poses']['a0::part_06'][0]+=.0001
    if fault in ('other_run','stale'):
        with pytest.raises(RuntimeError):guard.check(sample)
        return
    guard.check(sample)
    if fault=='no_motion':
        assert guard.release_candidate is None
        return
    if fault=='foreign_goal':
        with pytest.raises(RuntimeError,match='owned'):guard.terminal_release('b'*32,4,1,sample['wall_ns'])
    else:
        with pytest.raises(RuntimeError,match='successful'):guard.terminal_release(goal['uuid'],5,1,sample['wall_ns'])
    assert guard.ownership=='RELEASING'


def test_release_ready_consumes_pending_physical_separation_before_rejecting(tmp_path,monkeypatch):
    guard,sample,advance,position,goal,_=release_boundary_guard(tmp_path,monkeypatch)
    guard.terminal_release(goal['uuid'],4,1,sample['wall_ns']+1)
    advance();position[0]=0.;sample['contacts']=[]
    sample['poses']['a0::part_06'][0]+=.0001
    guard.m.drain=lambda:[copy.deepcopy(sample)]
    assert guard.release_candidate is None
    ready=guard.release_ready()
    assert ready['iteration']==sample['iteration']
    assert guard.release_candidate['iteration']==sample['iteration']
