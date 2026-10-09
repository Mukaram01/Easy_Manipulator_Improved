"""Actual nested Cartesian boundary and generic extraction handoff regressions."""
import ast
import copy
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT=Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py'
SPEC=importlib.util.spec_from_file_location('extraction_runtime',SCRIPT)
MODULE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODULE)


def straight_fixture(monkeypatch):
    from geometry_msgs.msg import PoseStamped,Pose
    from moveit_msgs.msg import MotionPlanRequest,PlanningScene,RobotState,RobotTrajectory,AttachedCollisionObject,CollisionObject
    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    p=Pose();p.orientation.w=1.
    target=CollisionObject(id='target',primitives=[SolidPrimitive(type=1,dimensions=[.025]*3)],primitive_poses=[p])
    initial=PlanningScene(robot_state=RobotState(joint_state=JointState(name=['arm'],position=[0.])))
    initial.world.collision_objects=[target]
    view=copy.deepcopy(initial);view.world.collision_objects=[]
    view.robot_state.attached_collision_objects=[AttachedCollisionObject(object=target)]
    start=PoseStamped();start.pose.orientation.w=1.
    intent={'schema':'workcell_extraction_intent/v1','variant_id':'geometry_away_0',
            'object_id':'target','candidate_id':'top_2f::000','offset_xyz_m':[-.002,0.,.1]}
    goal=MODULE.translated_pose(start,*intent['offset_xyz_m'])
    trajectory=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['arm'],points=[JointTrajectoryPoint(positions=[0.])]))
    submitted=[];audits=[]
    tree=ast.parse(SCRIPT.read_text()); main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    boundary=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='plan_segment')
    context=dict(vars(MODULE),MotionPlanRequest=MotionPlanRequest,GetStateValidity=GetStateValidity,
        stage=lambda name:None,deadline=time.monotonic()+10,contract={'planning_group':'arm','tool_link':'tcp'},
        args=SimpleNamespace(backend='simulator',segment_planning_time=3.),initial=initial,mimics=[],
        fk=lambda *a:copy.deepcopy(start),cell={'environment':{}},manifest={},
        legitimate_support_ids=lambda *a:[],call=lambda *a:SimpleNamespace(contacts=[]),validity_client=object(),
        summary={'normalized_objects':[]})
    exec(compile(ast.Module(body=[boundary],type_ignores=[]),'<actual-straight-lift>','exec'),context)
    original=context['plan_segment']
    def recurse(scene,name,adjusted,group,**kw):
        submitted.append((copy.deepcopy(adjusted),kw))
        return {'trajectory':trajectory,'after':copy.deepcopy(scene),'metadata':{}}
    context['plan_segment']=recurse
    import pile_extraction
    def audit(target,neighbors,selected,poses=None):
        audits.append(copy.deepcopy((target,neighbors,selected,poses)))
        return {'checked':True}
    monkeypatch.setattr(pile_extraction,'audit_extraction',audit)
    return original,view,goal,intent,submitted,audits


def test_selected_extraction_is_audited_before_and_after_fresh_cartesian_plan(monkeypatch):
    plan,view,goal,intent,submitted,audits=straight_fixture(monkeypatch)
    result=plan(view,'PREPLAN_LIFT',goal,straight=True,extraction_intent=intent)
    assert len(submitted)==1
    assert submitted[0][0]==goal
    assert submitted[0][1]['cartesian_corridor'][1][:3]==[-.002,0.,.1]
    assert submitted[0][1]['initial_separation_object_ids']==[]
    assert len(audits)==2 and audits[0][3] is None and audits[1][3]
    assert result['metadata']['extraction_intent']==intent
    assert result['metadata']['extraction_geometry_audit']=={'checked':True}


def test_rejected_extraction_geometry_never_submits_moveit_plan(monkeypatch):
    plan,view,goal,intent,submitted,audits=straight_fixture(monkeypatch)
    import pile_extraction
    def reject(*a,**kw):raise RuntimeError('near-only pair cannot clear before height limit')
    monkeypatch.setattr(pile_extraction,'audit_extraction',reject)
    with pytest.raises(RuntimeError,match='cannot clear'):
        plan(view,'PREPLAN_LIFT',goal,straight=True,extraction_intent=intent)
    assert submitted==[]


def test_selected_extraction_cannot_disagree_with_cartesian_goal(monkeypatch):
    plan,view,goal,intent,submitted,audits=straight_fixture(monkeypatch)
    goal.pose.position.x=.002
    with pytest.raises(RuntimeError,match='extraction.*goal'):
        plan(view,'PREPLAN_LIFT',goal,straight=True,extraction_intent=intent)
    assert submitted==[]


@pytest.mark.parametrize('attempts,expected',[
    ([{'failure_kind':'planning','moveit_code':-6}, {'failure_kind':'extraction'}],True),
    ([{'failure_kind':'planning','moveit_code':-2}, {'failure_kind':'extraction'}],True),
    ([{'failure_kind':'extraction'}, {'failure_kind':'collision','moveit_code':-2}],False)])
def test_earlier_variant_stochastic_failure_retains_existing_bounded_retry(attempts,expected):
    result=SimpleNamespace(success=False,reason_code='NO_VALID_EXTRACTION',extraction_attempts=attempts,
        checks=[{'status':'FAIL','failure_kind':'extraction','moveit_code':None}])
    assert MODULE.preplan_retryable_failure(result) is expected


def test_global_budget_failure_cannot_be_retried_due_to_earlier_variant():
    result=SimpleNamespace(success=False,reason_code='SEARCH_BUDGET_EXHAUSTED',
        extraction_attempts=[{'failure_kind':'planning','moveit_code':-6}],
        checks=[{'status':'FAIL','failure_kind':'budget'}])
    assert not MODULE.preplan_retryable_failure(result)


def test_post_plan_extraction_audit_rejects_unsafe_actual_cartesian_path(monkeypatch):
    plan,view,goal,intent,submitted,audits=straight_fixture(monkeypatch)
    import pile_extraction
    def audit(target,neighbors,selected,poses=None):
        if poses is not None:raise RuntimeError('actual path creates a foreign contact')
        return {'checked':True}
    monkeypatch.setattr(pile_extraction,'audit_extraction',audit)
    with pytest.raises(RuntimeError,match='foreign contact'):
        plan(view,'PREPLAN_LIFT',goal,straight=True,extraction_intent=intent)
    assert len(submitted)==1


def evaluator_boundary(**context):
    tree=ast.parse(SCRIPT.read_text())
    authored=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='plan_authored_cycle')
    evaluate=next(n for n in authored.body if isinstance(n,ast.FunctionDef) and n.name=='evaluate_once')
    namespace=dict(vars(MODULE),**context)
    exec(compile(ast.Module(body=[evaluate],type_ignores=[]),'<actual-candidate-evaluator>','exec'),namespace)
    return namespace['evaluate_once']


def test_resolved_cycle_requires_selected_extraction_before_any_planning():
    evaluate=evaluator_boundary(contract={},deadline=time.monotonic()+10,resolved={'saved':True},
        operations=SimpleNamespace(extraction_candidates=lambda *a:[]))
    result=evaluate({'approach_ik':{'bound':True},'transfer_ik_seed':{'bound':True}},
        search_pass='revalidate',planning_attempts=3,segment_time=3.)
    assert result['success'] is False
    assert result['reason_code']=='TASK_EXTRACTION_UNBOUND'
    assert result['retryable'] is False


def test_failed_variant_cannot_export_transfer_seed_to_another_variant_retry():
    result=SimpleNamespace(success=False,candidate_id='grasp::0',reason_code='PREPLAN_PLACE_FAILED',
        reason='placement collision',cycle=None,extraction_attempts=[],checks=[{'status':'FAIL'}],
        stages=[{'stage':'PREPLAN_APPROACH','success':True,'approach_ik':{'proven':'approach'}},
                {'stage':'PREPLAN_TRANSFER','success':True,'transfer_ik_seed':{'wrong_variant':'seed'}}])
    request={'intent':{'pick':{'grasp':{'lift':{'distance_m':.1}}},'place':{'placement':{
        'approach':{'distance_m':.1},'retreat':{'distance_m':.1},'clearance_m':.001}}},
        'observation':{'id':'target'},'candidate':object(),'destination':{}}
    evaluate=evaluator_boundary(contract={},deadline=time.monotonic()+10,resolved=None,
        resolution_reference_time=time.time(),intent={'pick':{'selection':{'object_filter':{'max_age_seconds':60.}}}},
        operations=object(),initial_scene=object(),summary={'candidate_attempts':[]},cycles={},
        preplan_full_cycle=lambda **kw:result)
    outcome=evaluate(request,search_pass='discovery',planning_attempts=1,segment_time=.75)
    assert outcome['approach_ik']=={'proven':'approach'}
    assert outcome['transfer_ik_seed'] is None
    assert outcome['extraction_intent'] is None
