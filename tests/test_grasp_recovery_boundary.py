"""Deterministic boundary tests; no fixture is physical acceptance evidence."""
import ast
import copy
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as N

import pytest
sys.path.insert(0,str(Path(__file__).parents[1] / "scripts"))
import simulator_execution as guards

SCRIPT = Path(__file__).parents[1] / 'scripts/perceived_object_grasp_execute.py'
SPEC = importlib.util.spec_from_file_location('recovery_executor', SCRIPT)
executor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(executor)


def recovery_type():
    assert hasattr(guards, 'RetentionRecovery'), 'typed retention recovery boundary is missing'
    return guards.RetentionRecovery


def loss():
    held = guards.HeldObject([0.,0.,0.,0.,0.,0.,1.], [0.,0.,0.,0.,0.,0.,1.],
                             {'left','right'}, {'left','right'}, .3, 0.)
    with pytest.raises(RuntimeError) as failure:
        held.check([0.,0.,0.,0.,0.,0.,1.], [0.,0.,0.,0.,0.,0.,1.], {'right'})
    return failure.value


def test_contact_loss_is_typed_without_relaxing_existing_guard():
    exc = loss()
    assert type(exc).__name__ == 'GraspRetentionLoss'
    assert str(exc) == 'measured grasp retention lost: contact/slip'
    assert exc.details['missing_contacts'] == ['left']
    assert exc.details['translation_error_m'] == 0.


def test_loss_revokes_cycle_and_keeps_only_historical_provenance():
    cls = recovery_type()
    cycle = dict(full_cycle_prevalidated=True, candidate={'grasp': [1,2,3]},
                 steps=[{'trajectory': object()}], object_id='runtime::box', extraction_intent={'id':'away'})
    summary = dict(full_cycle_prevalidated=True, selected_object_id='runtime::box',
                   selected_grasp_index=7, resolution_sha256='old-resolution')
    recovery = cls(cycle, summary, loss(), during_action=True)
    assert recovery.evidence['state'] == 'CANCEL_AND_STOP'
    assert summary['grasp_recovery'] is recovery.evidence
    assert cycle == {'full_cycle_prevalidated': False, 'invalidated_by': 'GRASP_RETENTION_LOSS'}
    assert not summary['full_cycle_prevalidated']
    assert recovery.evidence['previous_attempt']['resolution_sha256'] == 'old-resolution'
    assert recovery.evidence['retry_count'] == 0
    assert recovery.evidence['retry_limit'] == 1
    with pytest.raises(RuntimeError, match='prevalidated'):
        executor.require_prevalidated_execution(True, cycle)


def test_owned_cancel_must_finish_before_reconciliation():
    recovery = recovery_type()({}, {}, loss(), during_action=True)
    with pytest.raises(RuntimeError, match='CANCELLATION_UNCONFIRMED'):
        recovery.confirm_stop(dict(iteration=30,sim_ns=30,wall_ns=30), False)
    assert recovery.evidence['state'] == 'CANCEL_AND_STOP'
    recovery.confirm_stop(dict(iteration=30,sim_ns=30,wall_ns=30), True)
    assert recovery.evidence['state'] == 'FAILURE_RECONCILE'


@pytest.mark.parametrize('valid,physical_contacts,code', [
    (True, [], 'RECOVERY_RETREAT_POLICY_UNQUALIFIED'),
    (False, [], 'RECOVERY_CURRENT_STATE_INVALID'),
    (True, [{'a':'robot','b':'box'}], 'RECOVERY_START_STATE_IN_CONTACT'),
])
def test_retreat_qualification_never_authorizes_unqualified_motion(valid, physical_contacts, code):
    recovery = recovery_type()({}, {}, loss(), during_action=False)
    recovery.confirm_stop(dict(iteration=30,sim_ns=30,wall_ns=30), False)
    recovery.reconciled({'attached_ids': [], 'acm_restored': True, 'measured_geometry_matches': True})
    recovery.qualify_retreat(valid, physical_contacts)
    assert recovery.evidence['state'] == 'RECOVERY_BLOCKED'
    assert recovery.evidence['failure_code'] == code
    assert recovery.evidence['retry_count'] == 0
    assert recovery.evidence['transitions'] == [
        'CANCEL_AND_STOP', 'FAILURE_RECONCILE', 'QUALIFY_RETREAT', 'RECOVERY_BLOCKED']


def nested(name, context):
    main = next(n for n in ast.parse(SCRIPT.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    fn = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(vars(executor), **context)
    exec(compile(ast.Module(body=[fn],type_ignores=[]), str(SCRIPT), 'exec'), namespace)
    return namespace[name]


def test_old_trajectory_alias_cannot_be_submitted_after_invalidation():
    # The loop may still hold a step alias after clearing its enclosing cycle.
    summary = {'grasp_recovery': {'state': 'RECOVERY_BLOCKED'}}
    submissions = []
    client = N(wait_for_server=lambda **kw: True, send_goal_async=lambda g: submissions.append(g))
    action = nested('action', dict(summary=summary, execute_client=client, contact_guard=None,
                                   controlled_cancel=False, measurements=None, controller_audit=None))
    with pytest.raises(RuntimeError, match='RECOVERY_EXECUTION_INVALIDATED'):
        action(client, object(), 1)
    assert submissions == []


def scene_fixture():
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    initial = PlanningScene()
    initial.robot_state.joint_state = JointState(name=['arm','finger'],position=[0.,0.],velocity=[0.,0.])
    for name in ['target','neighbor']:
        obj = CollisionObject(id='runtime::'+name)
        obj.header.frame_id='world'; obj.pose.orientation.w=1.
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.025]*3)]
        obj.primitive_poses=[Pose()];obj.primitive_poses[0].orientation.w=1.
        initial.world.collision_objects.append(obj)
    current = copy.deepcopy(initial)
    current.robot_state.joint_state.position=[.8,.3]
    current.robot_state.attached_collision_objects=[AttachedCollisionObject(
        link_name='tool',object=copy.deepcopy(initial.world.collision_objects[0]))]
    current.world.collision_objects=current.world.collision_objects[1:]
    sample=dict(run_id='run',pid=99,iteration=40,sim_ns=40000000,wall_ns=40000000,
                joints={'arm':[.9,0.],'finger':[.3,0.]},
                poses={'target':[1.,2.,3.,0.,0.,0.,1.],'neighbor':[4.,5.,6.,0.,0.,0.,1.]},contacts=[])
    m=N(joints=lambda s:s['joints'],object_pose=lambda s,n:s['poses'][n])
    guard=N(object='runtime::target',pile_objects={f'runtime::{n}':{'name':n,'size':[.025]*3} for n in ('target','neighbor')})
    return initial,current,sample,m,guard


def test_reconciliation_uses_current_measured_joints_and_every_object_pose():
    assert hasattr(executor, 'retention_reconciliation_scene'), 'full measured reconciliation is missing'
    initial,current,sample,m,guard=scene_fixture()
    diff,expected=executor.retention_reconciliation_scene(initial,current,initial.allowed_collision_matrix,m,guard,sample)
    assert list(expected.robot_state.joint_state.position)==[.9,.3]
    assert list(expected.robot_state.joint_state.velocity)==[0.,0.]
    assert not expected.robot_state.attached_collision_objects
    assert expected.allowed_collision_matrix==initial.allowed_collision_matrix
    assert {o.id:executor.pose_values(o.pose) for o in expected.world.collision_objects}=={
        'runtime::target':sample['poses']['target'],'runtime::neighbor':sample['poses']['neighbor']}
    from moveit_msgs.msg import CollisionObject
    assert diff.robot_state.attached_collision_objects[0].object.operation==CollisionObject.REMOVE
    assert len(diff.world.collision_objects)==2
    assert list(initial.robot_state.joint_state.position)==[0.,0.]
    assert len(current.robot_state.attached_collision_objects)==1


@pytest.mark.parametrize('fault', ['missing_joint','missing_object','unknown_attachment','moving_joint'])
def test_reconciliation_rejects_incomplete_or_unsettled_current_state(fault):
    assert hasattr(executor, 'retention_reconciliation_scene'), 'full measured reconciliation is missing'
    initial,current,sample,m,guard=scene_fixture()
    if fault=='missing_joint':del sample['joints']['arm']
    if fault=='missing_object':del sample['poses']['neighbor']
    if fault=='unknown_attachment':current.robot_state.attached_collision_objects[0].object.id='unknown'
    if fault=='moving_joint':sample['joints']['arm'][1]=.002
    with pytest.raises(RuntimeError):
        executor.retention_reconciliation_scene(initial,current,initial.allowed_collision_matrix,m,guard,sample)


def runtime_reconcile_fixture(*, valid=True, contacts=None):
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import GetStateValidity
    initial,current,sample,m,guard=scene_fixture()
    guard.held=object();guard.planning_attached=True;guard.separation=object();guard.held_proof={'old':True}
    guard.ownership='CARRIED';guard.phase='lift'
    m.receipt={'run_id':'run','pid':99,'world':'world','model':'robot'}
    m.fresh=lambda:sample
    sample['contacts']=contacts or []
    stopped=copy.deepcopy(sample)
    for k in ('iteration','sim_ns','wall_ns'):stopped[k]-=1
    summary={'motion_stop_verified':True,'stopped_measurement':stopped,'cancellation_confirmed':True}
    recovery=recovery_type()({},summary,loss(),during_action=True)
    queries=[];updates=[]
    live=[current]
    def apply(diff):
        updates.append(copy.deepcopy(diff))
        # Apply the supplied changes; the live state already measures these joints.
        live[0].allowed_collision_matrix=copy.deepcopy(diff.allowed_collision_matrix)
        if diff.world.collision_objects:
            live[0].world.collision_objects=copy.deepcopy(diff.world.collision_objects)
            live[0].robot_state.attached_collision_objects=[]
            live[0].robot_state.joint_state.position=[.9,.3]
    def call(client,req):
        queries.append(copy.deepcopy(req))
        return GetStateValidity.Response(valid=valid)
    context=dict(recovery=recovery, summary=summary, measurements=m, contact_guard=guard,
        initial=initial, baseline=initial.allowed_collision_matrix, PlanningScene=PlanningScene,
        GetStateValidity=GetStateValidity, validity_client=object(), call=call, apply=apply,
        scene_now=lambda:copy.deepcopy(live[0]),evidence_scene=lambda *a:None)
    reconcile=nested('reconcile_retention_loss',context)
    return reconcile,recovery,summary,sample,guard,queries,updates


def test_runtime_reconciliation_qualifies_only_fresh_state_and_is_idempotent():
    reconcile,recovery,summary,sample,guard,queries,updates=runtime_reconcile_fixture()
    reconcile()
    evidence=summary['measured_reconciliation']
    assert evidence['iteration']==sample['iteration']
    assert evidence['robot_state_sha256'] and evidence['scene_sha256'] and evidence['sample_sha256']
    assert evidence['attached_ids']==[] and evidence['acm_restored']
    assert list(queries[0].robot_state.joint_state.position)==[.9,.3]
    assert not queries[0].robot_state.attached_collision_objects
    assert guard.held is None and not guard.planning_attached and guard.separation is None
    assert guard.ownership=='RECOVERY_UNHELD'
    assert recovery.evidence['failure_code']=='RECOVERY_RETREAT_POLICY_UNQUALIFIED'
    before=(len(updates),len(queries),copy.deepcopy(recovery.evidence))
    reconcile()
    assert (len(updates),len(queries),recovery.evidence)==before


@pytest.mark.parametrize('fault', ['old_sample','different_run','different_pid','stop_missing'])
def test_runtime_reconciliation_rejects_stale_or_unbound_measurement(fault):
    reconcile,recovery,summary,sample,guard,queries,updates=runtime_reconcile_fixture()
    if fault=='old_sample':sample['iteration']=summary['stopped_measurement']['iteration']
    if fault=='different_run':sample['run_id']='other'
    if fault=='different_pid':sample['pid']=100
    if fault=='stop_missing':summary['motion_stop_verified']=False
    with pytest.raises(RuntimeError):reconcile()
    assert queries==[]
    assert not any(d.world.collision_objects for d in updates)
    assert recovery.evidence['state']=='RECOVERY_BLOCKED'


def test_runtime_physical_contact_blocks_even_if_moveit_valid():
    contact={'a':'world::robot::finger::collision','b':'world::target::link::collision'}
    reconcile,recovery,_,_,_,_,_=runtime_reconcile_fixture(contacts=[contact])
    reconcile()
    assert recovery.evidence['failure_code']=='RECOVERY_START_STATE_IN_CONTACT'


def test_guard_preserves_typed_failure_sample(tmp_path,monkeypatch):
    from test_simulator_execution import provisional_pile_guard
    guard,sample,advance,_=provisional_pile_guard(tmp_path,monkeypatch)
    guard.begin_pile_admission()
    guard.establish()
    sample['contacts']=sample['contacts'][1:]
    with pytest.raises(guards.GraspRetentionLoss) as failure:guard.check(sample)
    assert failure.value.sample==sample
    original=copy.deepcopy(failure.value.sample)
    advance()
    assert failure.value.sample==original


def test_stop_certificate_must_postdate_the_loss_sample():
    failure=loss()
    failure.sample=dict(run_id='run',pid=99,iteration=50,sim_ns=50,wall_ns=50)
    recovery=recovery_type()({}, {}, failure, during_action=False)
    with pytest.raises(RuntimeError,match='RECOVERY_STOP_PRECEDES_LOSS'):
        recovery.confirm_stop(dict(failure.sample), False)
    assert recovery.evidence['state']=='CANCEL_AND_STOP'


def test_outer_hold_failure_invalidates_previous_stop_certificate():
    main=next(n for n in ast.parse(SCRIPT.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
    begin=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='begin_recovery')
    wrapper=ast.parse('def create(summary, cycle):\n    recovery=None\n    return begin_recovery\n').body[0]
    wrapper.body.insert(1,begin)
    context=dict(vars(executor))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),str(SCRIPT),'exec'),context)
    summary=dict(current_stage='HOLD',motion_stop_verified=True,stopped_measurement={'iteration':1},
                 cancellation_confirmed=True)
    begin=context['create'](summary, {'full_cycle_prevalidated':True})
    begin(loss())
    assert summary['motion_stop_verified'] is False
    assert summary['cancellation_confirmed'] is False
    assert 'stopped_measurement' not in summary
    # The outer catch of an action failure must not reset that action's new proof.
    summary['motion_stop_verified']=True
    begin(loss())
    assert summary['motion_stop_verified'] is True


def test_reconciliation_rejects_incomplete_live_robot_joint_names():
    initial,current,sample,m,guard=scene_fixture()
    current.robot_state.joint_state.name=['finger']
    current.robot_state.joint_state.position=[.3]
    with pytest.raises(RuntimeError,match='RECOVERY_CURRENT_JOINT_STATE_INVALID'):
        executor.retention_reconciliation_scene(initial,current,initial.allowed_collision_matrix,m,guard,sample)
