"""Behavioral extraction oracle: losing a transition invalidates a full cycle.

Only MoveIt service/action responses are simulated. Geometry, private scene
transitions, contact validation and containment use the production helpers.
"""
import copy
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import perceived_object_grasp_execute as runtime

EXPECTED = ['PREPLAN_APPROACH', 'PREPLAN_GRASP', 'PREPLAN_CLOSE_GRIPPER', 'ATTACH',
            'PREPLAN_LIFT', 'PREPLAN_TRANSFER', 'PREPLAN_PLACE', 'PREPLAN_OPEN_GRIPPER',
            'DETACH', 'PREPLAN_RETREAT', 'PREPLAN_HOME']


def fixture(fail_at=None, contacts=True):
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import PlanningScene, CollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import PreplanOperations
    observation = dict(id='observed-box', frame_id='world', dimensions=[.04, .04, .1],
                       pose=[.4, -.2, .3, 0., 0., 0., 1.], timestamp=time.time())
    def pose(values):
        result = PoseStamped()
        result.header.frame_id = 'world'
        (result.pose.position.x, result.pose.position.y, result.pose.position.z,
         result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z,
         result.pose.orientation.w) = [float(x) for x in values]
        return result
    original = CollisionObject(id=observation['id'])
    original.header.frame_id = 'world'
    original.pose = pose(observation['pose']).pose
    original.primitives = [SolidPrimitive(type=1, dimensions=observation['dimensions'])]
    original.primitive_poses = [pose([0., 0., 0., 0., 0., 0., 1.]).pose]
    obstacle = copy.deepcopy(original); obstacle.id = 'obstacle'
    scene = PlanningScene()
    scene.world.collision_objects = [original, obstacle]
    scene.allowed_collision_matrix = AllowedCollisionMatrix(entry_names=['tip', 'palm', 'obstacle'],
        entry_values=[AllowedCollisionEntry(enabled=[False]*3) for _ in range(3)])
    names = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'gripper_finger1_joint']
    scene.robot_state.joint_state = JointState(name=names, position=[0., 0., .5, 0., 0., 0., 1., 0.])
    contract = dict(tool_link='tool', grasp_frame='grasp', allowed_touch_links=['tip'],
        home_joint_names=names[:7], home_joint_positions=[0., 0., .5, 0., 0., 0., 1.],
        max_age_seconds=60., approach_distance_m=.12, retreat_distance_m=.15, tcp_pose=[0., 0., 0., 0., 0., 0., 1.])
    trace, goals, validity = [], [], []
    def stage(name):
        trace.append(name)
    def segment(view, name, goal, group=None, straight=False, **options):
        if name == fail_at:
            raise RuntimeError('injected obstacle')
        goals.append((name, copy.deepcopy(goal), group, straight))
        after = copy.deepcopy(view)
        values = goal if isinstance(goal, dict) else dict(zip(names[:7], runtime.pose_values(goal.pose)))
        after.robot_state = runtime.updated_state(after.robot_state, values)
        return dict(kind='motion', stage=name, before=copy.deepcopy(view), after=after,
                    trajectory=NS(stage=name), metadata=dict(stage=name, success=True, moveit_code=1, points=2))
    def state_validity(state):
        validity.append(state.joint_state.position[-1])
        pairs = [NS(contact_body_1='observed-box', contact_body_2='tip')] if contacts and state.joint_state.position[-1] >= .02 else []
        return NS(valid=not pairs, contacts=pairs)
    def transition(name, function):
        def apply(*args):
            if name == fail_at:
                raise RuntimeError('injected obstacle')
            return function(*args)
        return apply
    operations = PreplanOperations(plan_segment=segment, fk=lambda state, link: pose(state.joint_state.position[:7]),
        state_validity=state_validity, updated_state=runtime.updated_state,
        pose_message=pose, translated_pose=runtime.translated_pose,
        target_contact_matrix=runtime.target_contact_matrix, verify_selected_contacts=runtime.verify_selected_contacts,
        private_attachment=transition('ATTACH', runtime.private_attachment), object_pose_after_motion=runtime.object_pose_after_motion,
        place_detachment_diff=transition('DETACH', runtime.place_detachment_diff), stage=stage)
    candidate = generate_strategy_candidates('top_2f', observation, {'approach_distance_m': .12})[3]
    kwargs = dict(initial_scene=scene, observation=observation, candidate=candidate,
        destination=dict(pose_xyz=[.6, .2, .3], pose_rpy=[0., 0., 0.], dimensions=[.3, .3, .3]),
        contract=contract, operations=operations, deadline=time.monotonic()+30)
    return kwargs, trace, goals, validity


def test_complete_cycle_preserves_scene_states_metadata_and_top_geometry():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, goals, validity = fixture()
    initial = copy.deepcopy(kwargs['initial_scene'])
    candidate = copy.deepcopy(kwargs['candidate'])
    result = preplan_full_cycle(**kwargs)
    assert result.success, result.reason
    assert result.candidate_id == 'top_2f::003'
    assert result.reason_code is None
    assert result.cycle['candidate'] == candidate
    assert kwargs['candidate'] == candidate
    assert kwargs['initial_scene'] == initial
    steps = result.cycle['steps']
    assert [s['stage'] for s in steps] == EXPECTED
    assert len(result.stages) == 9
    assert trace == ['GENERATE_GRASPS'] + EXPECTED + ['CANDIDATE_READY']
    assert validity == pytest.approx([.01005, .0201])
    # Every authored z-axis approach/departure is a straight Cartesian segment, the place descent included.
    assert [g[0] for g in goals if g[3]] == ['PREPLAN_GRASP', 'PREPLAN_LIFT', 'PREPLAN_PLACE', 'PREPLAN_RETREAT']
    assert runtime.pose_values(goals[0][1].pose) == pytest.approx([.4, -.2, .47, -.7071067811865475, .7071067811865476, 0., 0.])
    assert [goals[i][1].pose.position.z for i in (3, 4, 5, 7)] == pytest.approx([.5, .5, .35, .5])
    assert steps[3]['original'] == initial.world.collision_objects[0]
    assert [o.id for o in steps[3]['after'].world.collision_objects] == ['obstacle']
    assert steps[3]['after'].allowed_collision_matrix == initial.allowed_collision_matrix
    for step in steps[4:8]:
        assert [o.object.id for o in step['before'].robot_state.attached_collision_objects] == ['observed-box']
    assert steps[8]['tool_at_grasp'].pose.position.z == pytest.approx(.35)
    assert steps[8]['after'].robot_state.attached_collision_objects == []
    assert [o.id for o in steps[-1]['after'].world.collision_objects] == ['obstacle', 'observed-box']
    assert steps[-1]['after'].allowed_collision_matrix == initial.allowed_collision_matrix
    assert steps[-1]['after'].robot_state.joint_state.position == initial.robot_state.joint_state.position
    assert any(c['code']=='DESTINATION_CONTAINMENT' and c['status']=='PASS' for c in result.checks)


@pytest.mark.parametrize('stage', EXPECTED)
def test_failure_at_any_transition_stops_without_a_cycle(stage):
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, _, _ = fixture(fail_at=stage)
    result = preplan_full_cycle(**kwargs)
    assert result.success is False
    assert result.cycle is None
    assert result.reason_code == stage + '_FAILED'
    assert result.reason == 'injected obstacle'
    assert trace[-1] == stage
    assert result.stages[-1]['stage'] == stage and not result.stages[-1]['success']


def test_planning_lifecycle_failure_at_approach_escapes_without_later_planning():
    from dataclasses import replace
    from full_cycle_preplanner import PlanningLifecycleFailure, preplan_full_cycle
    kwargs, trace, goals, validity = fixture()
    failure = PlanningLifecycleFailure('planning goal has no confirmed terminal state')
    calls = []
    def segment(*args, **options):
        calls.append(args[1])
        raise failure
    kwargs['operations'] = replace(kwargs['operations'], plan_segment=segment)
    with pytest.raises(PlanningLifecycleFailure) as caught:
        preplan_full_cycle(**kwargs)
    assert caught.value is failure
    assert calls == ['PREPLAN_APPROACH']
    assert trace == ['GENERATE_GRASPS', 'PREPLAN_APPROACH']
    assert goals == [] and validity == []


def test_missing_contact_never_passes_on_validity_alone():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, validity = fixture(contacts=False)
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code == 'PREPLAN_CLOSE_GRIPPER_FAILED'
    assert 'no allowed fingertip contact' in result.reason
    assert len(validity) == 80


def closure_fixture(contact_links_at):
    from dataclasses import replace
    kwargs, trace, goals, validity = fixture()
    kwargs['contract'].update(simulator_closure_reserve=True, allowed_touch_links=['left_tip', 'right_tip'])
    kwargs['initial_scene'].allowed_collision_matrix.entry_names=['left_tip','right_tip','obstacle']
    def state_validity(state):
        position=state.joint_state.position[-1]
        validity.append(position)
        links=contact_links_at(position)
        pairs=[NS(contact_body_1='observed-box', contact_body_2=link,
                  body_type_1=1, body_type_2=0, depth=.00001) for link in links]
        return NS(valid=not pairs, contacts=pairs)
    kwargs['operations']=replace(kwargs['operations'], state_validity=state_validity,
        screen_closure=lambda **kw:dict(accepted=True,neighbor_inflation_m={}))
    return kwargs, trace, goals, validity


def test_simulator_closure_refines_opposing_contact_then_checks_full_reserve():
    from full_cycle_preplanner import preplan_full_cycle
    from dataclasses import replace
    from test_grasp_capture import case
    qualify,data=case()
    kwargs, _, goals, validity=closure_fixture(
        lambda q:['left_tip','right_tip'] if q>=.012 else ['left_tip'] if q>=.008 else [])
    qualified=[]
    def qualify_planned_closure(**kw):
        qualified.append(kw['planned_closed'].joint_state.position[-1])
        return qualify(**data)
    kwargs['operations']=replace(kwargs['operations'],qualify_grasp=qualify_planned_closure)
    result=preplan_full_cycle(**kwargs)
    assert result.success, result.reason
    assert validity[:2]==pytest.approx([.01005,.0201])
    assert len(validity)==13  # Two coarse samples, ten refinements, endpoint.
    close=next(goal for name,goal,_,_ in goals if name=='PREPLAN_CLOSE_GRIPPER')
    metadata=next(s for s in result.stages if s['stage']=='PREPLAN_CLOSE_GRIPPER')
    lower,upper=metadata['first_opposing_bracket_rad']
    assert lower<.012<=upper
    assert upper-lower==pytest.approx(.01005/1024)
    assert metadata['contact_refinement_iterations']==10
    assert metadata['first_opposing_position_rad']==upper
    assert close['gripper_finger1_joint']==pytest.approx(upper+.01005)
    assert metadata['commanded_position_rad']==close['gripper_finger1_joint']
    assert validity[-1]==close['gripper_finger1_joint']
    assert qualified==[close['gripper_finger1_joint']]
    from full_cycle_preplanner import SIMULATOR_CLOSURE_RESERVE_RAD
    assert metadata['position_reserve_rad']==pytest.approx(SIMULATOR_CLOSURE_RESERVE_RAD)==pytest.approx(.01005)
    assert metadata['required_contact_links']==['left_tip','right_tip']
    assert metadata['planned_contact_links']==['left_tip','right_tip']


@pytest.mark.parametrize('reserve_links,reason', [
    (['left_tip','right_tip','forearm'], 'non-contact link'),
    (['left_tip'], 'opposing'),
    ([], 'opposing'),
])
def test_simulator_closure_rejects_forbidden_or_lost_contacts_at_reserve(reserve_links,reason):
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, goals, validity=closure_fixture(
        lambda q:reserve_links if q>.012 else ['left_tip','right_tip'] if q>=.007 else [])
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code=='PREPLAN_CLOSE_GRIPPER_FAILED'
    assert reason in result.reason.lower()
    assert validity[:2]==pytest.approx([.01005,0.])
    assert len(validity)==13
    assert .01705<=validity[-1]<=.01705+.01005/1024
    assert all(name!='PREPLAN_CLOSE_GRIPPER' for name,_,_,_ in goals)
    assert 'ATTACH' not in trace


def test_simulator_closure_rejects_opposing_contact_at_limit_without_reserve():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, _, validity=closure_fixture(
        lambda q:['left_tip','right_tip'] if q>=.804 else ['left_tip'])
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code=='PREPLAN_CLOSE_GRIPPER_FAILED'
    assert 'reserve' in result.reason
    assert len(validity)==90 and validity[79]==pytest.approx(.804)
    assert 'ATTACH' not in trace


@pytest.mark.parametrize('forbidden_position', [0., .015075])
def test_simulator_closure_rejects_forbidden_contact_during_refinement(forbidden_position):
    from full_cycle_preplanner import preplan_full_cycle
    def contacts(q):
        if q==pytest.approx(forbidden_position):
            return ['left_tip','right_tip','forearm']
        threshold=.007 if forbidden_position==0. else .012
        return ['left_tip','right_tip'] if q>=threshold else ['left_tip']
    kwargs, trace, goals, validity=closure_fixture(contacts)
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code=='PREPLAN_CLOSE_GRIPPER_FAILED'
    assert 'non-contact link' in result.reason
    assert validity[-1]==pytest.approx(forbidden_position)
    assert result.checks[-1]['failure_kind']=='collision'
    assert all(name!='PREPLAN_CLOSE_GRIPPER' for name,_,_,_ in goals)
    assert 'ATTACH' not in trace


@pytest.mark.parametrize('global_expired,reason_code', [
    (False, 'CANDIDATE_SLICE_EXHAUSTED'), (True, 'SEARCH_BUDGET_EXHAUSTED')])
def test_simulator_contact_refinement_checks_deadlines_after_each_query(
        monkeypatch, global_expired, reason_code):
    from dataclasses import replace
    import full_cycle_preplanner
    kwargs, trace, goals, validity=closure_fixture(
        lambda q:['left_tip','right_tip'] if q>=.012 else ['left_tip'])
    clock=[1000.]
    monkeypatch.setattr(full_cycle_preplanner, 'time', NS(monotonic=lambda:clock[0],time=time.time))
    kwargs['deadline']=1003.
    kwargs['contract']['search_deadline']=1003. if global_expired else 1004.
    original=kwargs['operations'].state_validity
    def delayed_query(state):
        response=original(state)
        clock[0]+=1.
        return response
    kwargs['operations']=replace(kwargs['operations'],state_validity=delayed_query)
    result=full_cycle_preplanner.preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code==reason_code
    assert validity==pytest.approx([.01005,.0201,.015075])
    assert all(name!='PREPLAN_CLOSE_GRIPPER' for name,_,_,_ in goals)
    assert 'ATTACH' not in trace


def test_default_closure_still_stops_at_first_single_allowed_contact():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, validity=fixture()
    result=preplan_full_cycle(**kwargs)
    assert result.success, result.reason
    assert validity==pytest.approx([.01005,.0201])
    assert next(goal for name,goal,_,_ in goals if name=='PREPLAN_CLOSE_GRIPPER')=={'gripper_finger1_joint':.0201}
    assert 'position_reserve_rad' not in next(s for s in result.stages if s['stage']=='PREPLAN_CLOSE_GRIPPER')


def test_destination_containment_fails_before_open_detach_or_success():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, _, _ = fixture()
    # Fits initial extents but fails the required 1 mm physical clearance.
    kwargs['destination']['dimensions'] = [.04, .04, .1]
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code == 'DESTINATION_CONTAINMENT_FAILED'
    assert 'PREPLAN_OPEN_GRIPPER' not in trace and 'CANDIDATE_READY' not in trace


def test_preplanner_operations_have_no_execution_capability():
    from dataclasses import fields
    from full_cycle_preplanner import PreplanOperations
    # The injected public capability contract is intentionally plan-only.
    assert {f.name for f in fields(PreplanOperations)} == {
        'plan_segment', 'fk', 'state_validity', 'updated_state', 'pose_message',
        'translated_pose', 'target_contact_matrix', 'verify_selected_contacts',
        'private_attachment', 'object_pose_after_motion', 'place_detachment_diff', 'stage',
        'extraction_candidates', 'qualify_grasp', 'screen_closure'}


@pytest.mark.parametrize('change,reason', [
    ('expired', 'observation expired'), ('deadline', 'slice exhausted'),
    ('identity', 'candidate object'), ('strategy', 'unsupported'),
    ('retreat', 'retreat distance'),
])
def test_invalid_preconditions_never_reach_planning(change, reason):
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs['contract']['max_age_seconds'] = 5.
    if change == 'expired':
        kwargs['observation']['timestamp'] -= 10.
    elif change == 'deadline':
        kwargs['deadline'] = time.monotonic()-1
    elif change == 'identity':
        kwargs['candidate'] = replace(kwargs['candidate'], object_id='different-track')
    elif change == 'strategy':
        kwargs['candidate'] = replace(kwargs['candidate'], strategy_ref='side_grip_basic')
    else:
        kwargs['contract']['retreat_distance_m'] = float('nan')
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert reason in result.reason
    assert goals == []


def test_candidate_slice_exhaustion_is_retryable_but_not_global_search_stop():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs["contract"]["search_deadline"] = time.monotonic() + 30.0
    kwargs["deadline"] = time.monotonic() - 0.001
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code == "CANDIDATE_SLICE_EXHAUSTED"
    assert result.checks[-1]["failure_kind"] == "budget"
    assert runtime.preplan_retryable_failure(result)
    assert goals == []


def test_failed_motion_evidence_cannot_become_prevalidated():
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, _ = fixture()
    plan = kwargs['operations'].plan_segment
    def failed(*args):
        step = plan(*args)
        step['metadata']['success'] = False
        return step
    kwargs['operations'] = replace(kwargs['operations'], plan_segment=failed)
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code == 'PREPLAN_APPROACH_FAILED'


def test_preplanner_does_not_claim_unconsumed_candidate_constraints():
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs['candidate'] = replace(kwargs['candidate'], effective={'approach_distance_m': .12, 'force_limit_n': 4.})
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert 'unsupported' in result.reason
    assert goals == []


def test_side_grip_candidate_is_consumed_as_a_lateral_physical_cycle():
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, trace, goals, _ = fixture()
    kwargs['candidate'] = generate_strategy_candidates(
        'side_grip_basic', kwargs['observation'], {'approach_distance_m': .08})[0]

    result = preplan_full_cycle(**kwargs)

    assert result.success
    assert result.candidate_id == 'side_grip_basic::000'
    assert trace == ['GENERATE_GRASPS'] + EXPECTED + ['CANDIDATE_READY']
    approach, contact = goals[:2]
    assert approach[0] == 'PREPLAN_APPROACH' and approach[1].pose.position.x == pytest.approx(.50)
    assert contact[0] == 'PREPLAN_GRASP' and contact[1].pose.position.x == pytest.approx(.42)
    assert contact[1].pose.position.y == pytest.approx(-.2)
    assert contact[1].pose.position.z == pytest.approx(.3)
    assert contact[3] is True
    assert result.cycle['candidate'].effective == {
        'approach_axis': 'x_plus',
        'orientation_mode': 'horizontal',
        'approach_distance_m': .08,
    }


def test_side_grip_preplanner_accepts_normalized_tuple_pose():
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, _ = fixture()
    kwargs['observation']['pose'] = tuple(kwargs['observation']['pose'])
    kwargs['candidate'] = generate_strategy_candidates(
        'side_grip_basic', kwargs['observation'], {'approach_distance_m': .08})[0]

    assert preplan_full_cycle(**kwargs).success


def test_finger_pinch_consumes_tilted_geometry_tcp_and_private_cycle():
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs['observation']['pose'][3:] = [0., .7071067811865475, 0., .7071067811865476]
    original = kwargs['initial_scene'].world.collision_objects[0]
    original.pose.orientation.y = .7071067811865475
    original.pose.orientation.w = .7071067811865476
    kwargs['contract']['tcp_pose'] = [0., 0., .02, 0., 0., 0., 1.]
    kwargs['candidate'] = generate_strategy_candidates(
        'finger_pinch_basic', kwargs['observation'], {'approach_distance_m': .07})[0]
    initial = copy.deepcopy(kwargs['initial_scene'])
    result = preplan_full_cycle(**kwargs)
    assert result.success, result.reason
    assert result.candidate_id == 'finger_pinch_basic::000'
    # World grasp contact .45 becomes tool .47: installed 2 cm TCP applied once.
    assert runtime.pose_values(goals[0][1].pose)[:3] == pytest.approx([.54, -.2, .3])
    assert runtime.pose_values(goals[1][1].pose)[:3] == pytest.approx([.47, -.2, .3])
    assert goals[1][3] is True
    assert [step['stage'] for step in result.cycle['steps']] == EXPECTED
    assert result.cycle['steps'][3]['after'].robot_state.attached_collision_objects
    assert not result.cycle['steps'][-1]['after'].robot_state.attached_collision_objects
    placed = result.cycle['steps'][-1]['after'].world.collision_objects[-1]
    assert runtime.pose_values(placed.pose)[:3] == pytest.approx([.6, .2, .3])
    assert kwargs['initial_scene'] == initial


@pytest.mark.parametrize('yaw_index,success', [(0, False), (1, True)])
def test_finger_pinch_aperture_follows_finger_closing_axis(yaw_index, success):
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, goals, _ = fixture()
    kwargs['observation']['dimensions'] = [.04, .10, .06]
    kwargs['initial_scene'].world.collision_objects[0].primitives[0].dimensions = [.04, .10, .06]
    kwargs['candidate'] = generate_strategy_candidates(
        'finger_pinch_basic', kwargs['observation'], {'approach_distance_m': .07})[yaw_index]
    result = preplan_full_cycle(**kwargs)
    assert result.success is success, result.reason
    if not success:
        assert 'aperture' in result.reason
        assert goals == []


@pytest.mark.parametrize('failed_stage', ['PREPLAN_APPROACH', 'PREPLAN_GRASP',
                                        'PREPLAN_CLOSE_GRIPPER', 'PREPLAN_LIFT'])
def test_full_path_rejection_keeps_collision_evidence_and_original_ranking(failed_stage):
    from dataclasses import replace
    kwargs, _, goals, _ = fixture()
    plan = kwargs['operations'].plan_segment
    attempts = {'count': 0}
    def collision_once(view, name, goal, group=None, straight=False):
        if name == 'PREPLAN_APPROACH':
            attempts['count'] += 1
        if attempts['count'] == 1 and name == failed_stage:
            error = RuntimeError('fixture collision along required motion')
            error.details = dict(failure_kind='collision', colliding_links=['generic_tool_link'],
                                 collision_objects=['generic_container'], moveit_code=-12)
            raise error
        return plan(view, name, goal, group, straight)
    kwargs['operations'] = replace(kwargs['operations'], plan_segment=collision_once)
    target = dict(kwargs.pop('observation'), confidence=.9)
    kwargs.pop('candidate')
    summary = {'candidate_attempts': []}
    cycle = runtime.plan_legacy_cycle(targets=[target], summary=summary, **kwargs)
    rejected, selected = summary['candidate_attempts']
    assert [rejected['grasp_index'], selected['grasp_index']] == [3, 0]
    assert rejected['failed_stage'] == failed_stage
    failure = rejected['checks'][-1]
    assert failure['candidate_id'] == 'top_2f::003'
    assert failure['failed_stage'] == failed_stage
    assert failure['failure_kind'] == 'collision'
    assert failure['colliding_links'] == ['generic_tool_link']
    assert failure['collision_objects'] == ['generic_container']
    assert failure['moveit_code'] == -12
    assert cycle['grasp_index'] == 0 and cycle['full_cycle_prevalidated']
    selected_goals = goals[-9:]
    assert all(next(g[3] for g in selected_goals if g[0] == name)
               for name in ['PREPLAN_GRASP', 'PREPLAN_LIFT'])


@pytest.mark.parametrize('policy,ready', [('AUTO', True), ('PREFERRED', True), ('EXACT', False)])
def test_authored_resolver_keeps_policy_and_descent_rejection_evidence(policy, ready):
    from dataclasses import replace
    from tests.test_task_intent_resolver import valid_intent, environment, cell
    from full_cycle_preplanner import MotionFeasibilityFailure
    from moveit_msgs.msg import ContactInformation
    kwargs, _, _, _ = fixture()
    observation = dict(kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate'); kwargs.pop('destination')
    intent = valid_intent(policy)
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2
    plan = kwargs['operations'].plan_segment
    count = {'approaches': 0}
    def first_descent_blocked(view, name, goal, group=None, straight=False):
        if name == 'PREPLAN_APPROACH':
            count['approaches'] += 1
        if name == 'PREPLAN_GRASP' and count['approaches'] == 1:
            raise MotionFeasibilityFailure('tool contacts container during descent', moveit_code=-12,
                contacts=[ContactInformation(contact_body_1='generic_container', body_type_1=1,
                    contact_body_2='generic_tool_link', body_type_2=0, depth=.001)])
        return plan(view, name, goal, group, straight)
    kwargs['operations'] = replace(kwargs['operations'], plan_segment=first_descent_blocked)
    summary = {'candidate_attempts': []}
    if ready:
        cycle = runtime.plan_authored_cycle(intent=intent, environment=env, cell=cell(),
            targets=[observation], summary=summary, **kwargs)
        assert cycle['candidate'].candidate_id == 'top_2f::001'
        assert cycle['full_cycle_prevalidated']
    else:
        with pytest.raises(RuntimeError, match='PREPLAN_GRASP_FAILED'):
            runtime.plan_authored_cycle(intent=intent, environment=env, cell=cell(),
                targets=[observation], summary=summary, **kwargs)
    resolution = summary['task_intent_resolution']
    assert resolution['readiness_status'] == ('READY' if ready else 'BLOCKED')
    grasp = resolution['grasp_resolution']
    assert [a['candidate_id'] for a in grasp['attempts']] == (['top_2f::000', 'top_2f::001'] if ready else ['top_2f::000'])
    assert not grasp['fallback']['used']
    failure = grasp['attempts'][0]['checks'][-1]
    assert failure['failed_stage'] == 'PREPLAN_GRASP'
    assert failure['failure_kind'] == 'collision'
    assert failure['colliding_links'] == ['generic_tool_link']
    assert failure['collision_objects'] == ['generic_container']


def test_authored_resolve_discovers_all_candidates_before_retrying_timeouts():
    """A stochastic timeout cannot monopolize the unresolved candidate search."""
    from dataclasses import replace
    from tests.test_task_intent_resolver import valid_intent, environment, cell
    from full_cycle_preplanner import MotionFeasibilityFailure

    kwargs, _, _, _ = fixture()
    observation = dict(kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate'); kwargs.pop('destination')
    intent = valid_intent('AUTO')
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2
    base_contract = kwargs['contract']
    plan = kwargs['operations'].plan_segment

    def transient_timeout(view, name, goal, group=None, straight=False):
        if (name == 'PREPLAN_APPROACH' and
                str(base_contract.get('_candidate_search_pass', '')).startswith('discovery:')):
            raise MotionFeasibilityFailure(
                'MoveIt action failed: status=6, code=-6', moveit_code=-6)
        return plan(view, name, goal, group, straight)

    kwargs['operations'] = replace(kwargs['operations'], plan_segment=transient_timeout)
    summary = {'candidate_attempts': []}
    cycle = runtime.plan_authored_cycle(
        intent=intent, environment=env, cell=cell(), targets=[observation],
        summary=summary, planning_time=3.0, **kwargs)

    # AUTO is strategy-phased: all eight top candidates get one cheap discovery
    # window, then the strongest top candidate is retried before side/pinch
    # discovery is allowed to spend budget.
    discovery = [a for a in summary['candidate_attempts']
                 if a['search_pass'].startswith('discovery:')]
    retries = [a for a in summary['candidate_attempts']
               if a['search_pass'].startswith('retry:')]
    assert len(discovery) == 8
    assert all(a['planning_attempts'] == 1 and a['segment_planning_time'] == .75
               for a in discovery)
    assert retries and retries[0]['candidate_id'] == 'top_2f::000'
    assert retries[0]['planning_attempts'] == 3
    assert cycle['candidate'].candidate_id == 'top_2f::000'
    search = summary['candidate_search']
    assert search['mode'] == 'strategy_phased_progress_beam'
    assert search['strategy_phases'][0] == 'top_2f'
    assert search['phases'][0]['discovered_candidates'] == 8
    assert search['phases'][0]['retryable_candidates'] == 8
    assert search['retry_beam_width'] == 3
    assert search['retry_pass_used'] is True


@pytest.mark.parametrize('retry_budget,search_budget,ready,reason_code', [
    (20.0, 1000.0, False, 'CANDIDATE_SLICE_EXHAUSTED'),
    (120.0, 1000.0, True, None),
    (120.0, 23.0, False, 'SEARCH_BUDGET_EXHAUSTED'),
])
def test_authored_retry_budget_covers_certificate_time_but_keeps_global_deadline(
        monkeypatch, retry_budget, search_budget, ready, reason_code):
    """A slow, valid approach must leave time for the remaining real cycle checks."""
    from dataclasses import replace
    import full_cycle_preplanner
    from full_cycle_preplanner import MotionFeasibilityFailure
    from tests.test_task_intent_resolver import valid_intent, environment, cell

    kwargs, _, goals, validity = fixture()
    observation = dict(kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate'); kwargs.pop('destination')
    initial_scene = copy.deepcopy(kwargs['initial_scene'])
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2
    clock = [1000.0]
    fake_time = NS(monotonic=lambda: clock[0], time=time.time)
    monkeypatch.setattr(runtime, 'time', fake_time)
    monkeypatch.setattr(full_cycle_preplanner, 'time', fake_time)
    kwargs['deadline'] = clock[0] + search_budget
    base_contract = kwargs['contract']
    plan = kwargs['operations'].plan_segment

    def slow_certified_approach(view, name, goal, group=None, straight=False):
        if name == 'PREPLAN_APPROACH':
            if base_contract['_candidate_search_pass'].startswith('discovery:'):
                raise MotionFeasibilityFailure('discovery planning timed out', moveit_code=-6)
            # Only the MoveIt boundary and elapsed clock are simulated. The
            # shared preplanner still checks deadlines, contact, attachment,
            # placement containment, detachment, retreat and home itself.
            clock[0] += 24.0
        return plan(view, name, goal, group, straight)

    kwargs['operations'] = replace(kwargs['operations'], plan_segment=slow_certified_approach)
    summary = {'candidate_attempts': []}

    def resolve():
        return runtime.plan_authored_cycle(
            intent=valid_intent('AUTO'), environment=env, cell=cell(),
            targets=[observation], summary=summary,
            retry_candidate_budget=retry_budget, **kwargs)

    if ready:
        cycle = resolve()
        assert cycle['candidate'].candidate_id == 'top_2f::000'
        assert cycle['full_cycle_prevalidated']
        assert [step['stage'] for step in cycle['steps']] == EXPECTED
        assert len(goals) == 9
        assert validity == pytest.approx([.01005, .0201])
        assert cycle['steps'][-1]['after'].robot_state == initial_scene.robot_state
        assert cycle['steps'][-1]['after'].allowed_collision_matrix == initial_scene.allowed_collision_matrix
    else:
        with pytest.raises(RuntimeError):
            resolve()
        assert validity == []
        assert all(goal[0] == 'PREPLAN_APPROACH' for goal in goals)

    attempts = summary['candidate_attempts']
    assert [(attempt['search_pass'], attempt['candidate_id']) for attempt in attempts[:8]] == [
        ('discovery:top_2f', f'top_2f::{index:03d}') for index in range(8)]
    assert all(attempt['candidate_wall_budget'] == .75 for attempt in attempts[:8])
    retry = attempts[8]
    assert retry['search_pass'] == 'retry:top_2f'
    assert retry['candidate_id'] == 'top_2f::000'
    assert retry['candidate_wall_budget'] == retry_budget
    assert retry['candidate_wall_seconds'] == 24.0
    assert retry['reason_code'] == reason_code
    assert retry['stages'][0]['stage'] == 'PREPLAN_APPROACH'
    assert retry['stages'][0]['success']
    if ready:
        assert len(attempts) == 9
        assert len(retry['stages']) == 9
        assert any(check['code'] == 'DESTINATION_CONTAINMENT' and check['status'] == 'PASS'
                   for check in retry['checks'])
    else:
        assert retry['stages'][-1]['stage'] == 'PREPLAN_GRASP'
        assert not retry['stages'][-1]['success']
        if reason_code == 'SEARCH_BUDGET_EXHAUSTED':
            assert len(attempts) == 9
            assert summary['task_intent_resolution']['readiness']['primary_code'] == reason_code
    assert kwargs['initial_scene'] == initial_scene


def timed_authored_fixture(monkeypatch, policy):
    """Real resolver/cycle transitions with a slow simulated MoveIt boundary."""
    from dataclasses import replace
    import full_cycle_preplanner
    from tests.test_task_intent_resolver import valid_intent, environment, cell
    kwargs, _, goals, validity=fixture()
    observation=dict(kwargs.pop('observation'),confidence=.9,class_id='bottle',shape='BOX')
    kwargs.pop('candidate');kwargs.pop('destination')
    env=environment()
    env['task_zones'][1]['placement_local']['dimensions'][2]=.2
    env['task_zones'][1]['dimensions'][2]=.2
    clock=[1000.]
    fake_time=NS(monotonic=lambda:clock[0],time=time.time)
    monkeypatch.setattr(runtime,'time',fake_time)
    monkeypatch.setattr(full_cycle_preplanner,'time',fake_time)
    contract=kwargs['contract']
    contract.update(robot_model_sha256='fixture',planning_group='arm')
    plan=kwargs['operations'].plan_segment
    settings=[]
    def timed_plan(view,name,goal,group=None,straight=False,**bindings):
        settings.append((contract['_candidate_search_pass'],
                         contract['_candidate_planning_attempts'],contract['_candidate_planning_time']))
        clock[0]+=24. if name=='PREPLAN_APPROACH' else 1.
        step=plan(view,name,goal,group,straight)
        positions=dict(zip(step['after'].robot_state.joint_state.name,
                           step['after'].robot_state.joint_state.position))
        for stage,key,option,binder in (
                ('PREPLAN_APPROACH','approach_ik','ik_binding',runtime.approach_ik_binding),
                ('PREPLAN_TRANSFER','transfer_ik_seed','ik_seed',runtime.transfer_ik_seed)):
            if name==stage:
                binding=binder(goal,contract,positions)
                if option in bindings:
                    assert bindings[option]==binding
                step['metadata'][key]=binding
        return step
    kwargs['operations']=replace(kwargs['operations'],plan_segment=timed_plan)
    kwargs.update(intent=valid_intent(policy),environment=env,cell=cell(),targets=[observation],
                  summary={'candidate_attempts':[]},retry_candidate_budget=120.)
    return kwargs,clock,goals,validity,settings


@pytest.mark.parametrize('policy,discovery_budget,search_budget,ready,attempt_count', [
    ('AUTO',None,1000.,True,9),
    ('AUTO',60.,1000.,True,1),
    ('AUTO',60.,23.,False,1),
    ('EXACT',None,1000.,False,1),
    ('EXACT',60.,1000.,True,1),
])
def test_discovery_wall_budget_allows_certification_without_changing_planner_or_policy(
        monkeypatch,policy,discovery_budget,search_budget,ready,attempt_count):
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,policy)
    initial=copy.deepcopy(kwargs['initial_scene'])
    kwargs['deadline']=clock[0]+search_budget
    if discovery_budget is not None:
        kwargs['discovery_candidate_budget']=discovery_budget
    if ready:
        cycle=runtime.plan_authored_cycle(**kwargs)
        assert cycle['candidate'].candidate_id=='top_2f::000'
        assert cycle['full_cycle_prevalidated']
        assert [step['stage'] for step in cycle['steps']]==EXPECTED
        assert validity==pytest.approx([.01005,.0201])
        assert cycle['steps'][-1]['after'].robot_state==initial.robot_state
        assert cycle['steps'][-1]['after'].allowed_collision_matrix==initial.allowed_collision_matrix
        assert any(check['code']=='DESTINATION_CONTAINMENT' and check['status']=='PASS'
                   for check in kwargs['summary']['candidate_attempts'][-1]['checks'])
    else:
        with pytest.raises(RuntimeError):
            runtime.plan_authored_cycle(**kwargs)
        expected='SEARCH_BUDGET_EXHAUSTED' if search_budget==23. else 'CANDIDATE_SLICE_EXHAUSTED'
        assert kwargs['summary']['task_intent_resolution']['readiness']['primary_code']==expected
        assert validity==[]
        assert [goal[0] for goal in goals]==['PREPLAN_APPROACH']
    attempts=kwargs['summary']['candidate_attempts']
    assert len(attempts)==attempt_count
    discovery=[item for item in attempts if item['search_pass'].startswith('discovery:')]
    assert [item['candidate_id'] for item in discovery]==[
        f'top_2f::{index:03d}' for index in range(len(discovery))]
    assert all(item['candidate_wall_budget']==(discovery_budget or .75) for item in discovery)
    assert all(attempts==1 and planning_time==.75 for phase,attempts,planning_time in settings
               if phase.startswith('discovery:'))
    assert kwargs['summary']['candidate_search']['retry_beam_width']==3
    if discovery_budget is None and policy=='AUTO':
        assert attempts[-1]['search_pass']=='retry:top_2f'
        assert all(item['reason_code']=='CANDIDATE_SLICE_EXHAUSTED' for item in discovery)
    else:
        assert not kwargs['summary']['candidate_search']['retry_pass_used']
    assert kwargs['initial_scene']==initial


@pytest.mark.parametrize('discovery_budget',[.75,60.])
def test_resolved_revalidation_uses_global_budget_and_same_bound_candidate(
        monkeypatch,discovery_budget):
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'AUTO')
    kwargs.update(deadline=clock[0]+1000.,discovery_candidate_budget=60.)
    selected=runtime.plan_authored_cycle(**kwargs)
    resolved=copy.deepcopy(kwargs['summary']['task_intent_resolution'])
    goals.clear();validity.clear();settings.clear()
    kwargs.update(deadline=clock[0]+1000.,discovery_candidate_budget=discovery_budget,
                  resolved=resolved,summary={'candidate_attempts':[]})
    revalidated=runtime.plan_authored_cycle(**kwargs)
    assert revalidated['candidate']==selected['candidate']
    assert [step['stage'] for step in revalidated['steps']]==EXPECTED
    assert len(goals)==9 and validity==pytest.approx([.01005,.0201])
    assert settings==[('revalidate',3,3.)]*9
    attempt,=kwargs['summary']['candidate_attempts']
    assert attempt['candidate_wall_budget'] is None
    assert attempt['candidate_wall_seconds']==32.
    assert kwargs['summary']['candidate_search']['mode']=='resolved_revalidation'
    assert kwargs['summary']['resolution_sha256']==resolved['resolution_sha256']


@pytest.mark.parametrize('budget',[0.,-1.,float('inf'),float('nan')])
def test_discovery_wall_budget_rejects_invalid_values(monkeypatch,budget):
    kwargs,_,goals,_,_=timed_authored_fixture(monkeypatch,'AUTO')
    with pytest.raises(ValueError,match='discovery budget must be finite and positive'):
        runtime.plan_authored_cycle(discovery_candidate_budget=budget,**kwargs)
    assert goals==[]


def test_retry_reservation_preserves_exact_first_candidate(monkeypatch):
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'EXACT')
    kwargs.update(deadline=clock[0]+1000.,discovery_candidate_budget=.75,
                  reserve_retry_budget=True)
    with pytest.raises(RuntimeError):
        runtime.plan_authored_cycle(**kwargs)
    attempt,=kwargs['summary']['candidate_attempts']
    assert attempt['candidate_id']=='top_2f::000'
    assert attempt['search_pass']=='discovery:top_2f'
    assert attempt['reason_code']=='CANDIDATE_SLICE_EXHAUSTED'
    assert kwargs['summary']['candidate_search']['retry_pass_used'] is False
    assert settings==[('discovery:top_2f',1,.75)]
    assert [goal[0] for goal in goals]==['PREPLAN_APPROACH']
    assert validity==[]


@pytest.mark.parametrize('retry_budget,discovery_count',[(120.,14),(180.,12)])
def test_retry_reservation_leaves_time_for_deepest_candidate_and_complete_cycle(
        monkeypatch,retry_budget,discovery_count):
    """Unit replay of observed discovery costs; no runtime qualification claim.

    Durations are the first 12 attempts in the 2026-09-30 transfer-clearance
    Resolve summary. Its first object's candidates 001/003 reached transfer
    and lift with 7/5 passed checks, but all 900 seconds went to discovery.
    """
    import full_cycle_preplanner
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'PREFERRED')
    grasp=kwargs['intent']['pick']['grasp']
    grasp['strategy_ref']='finger_pinch_basic'
    grasp['approach']['axis']='tool_z'
    grasp['orientation']['mode']='tool_aligned'
    original=kwargs['targets'][0]
    kwargs['targets']=[original]+[
        dict(copy.deepcopy(original),id=f'observed-box-{index}') for index in range(1,4)]
    kwargs.update(deadline=1900.,discovery_candidate_budget=60.,reserve_retry_budget=True,
                  retry_candidate_budget=retry_budget)
    initial=copy.deepcopy(kwargs['initial_scene'])
    durations=[22.235,79.809,10.465,60.848,9.320,98.731,
               10.207,68.800,9.738,53.203,10.688,46.107]
    discovered=[]
    saved={}
    real_preplan=full_cycle_preplanner.preplan_full_cycle

    def observed_discovery_then_real_retry(**arguments):
        candidate=arguments['candidate']
        phase=kwargs['contract']['_candidate_search_pass']
        if phase.startswith('retry:'):
            assert phase=='retry:finger_pinch_basic'
            assert arguments['observation']['id']==original['id']
            assert candidate.candidate_id=='finger_pinch_basic::001'
            assert arguments['contract']['approach_ik']==saved[candidate.candidate_id]
            assert arguments['deadline']==pytest.approx(clock[0]+retry_budget)
            assert arguments['contract']['search_deadline']==1900.
            assert kwargs['contract']['_candidate_planning_attempts']==3
            assert kwargs['contract']['_candidate_planning_time']==3.
            return real_preplan(**arguments)
        index=len(discovered)
        discovered.append((arguments['observation']['id'],candidate.candidate_id))
        assert phase=='discovery:finger_pinch_basic'
        assert arguments['deadline']==pytest.approx(min(1900.,clock[0]+60.))
        assert kwargs['contract']['_candidate_planning_attempts']==1
        assert kwargs['contract']['_candidate_planning_time']==.75
        clock[0]+=durations[index] if index<len(durations) else 60.
        retryable=index in (1,3)
        stages=[]
        checks=[]
        failed='PREPLAN_TRANSFER' if index==1 else 'PREPLAN_LIFT'
        if retryable:
            target=arguments['operations'].pose_message(candidate.approach_pose)
            binding=runtime.approach_ik_binding(target,kwargs['contract'],
                dict(zip(kwargs['contract']['home_joint_names'],candidate.approach_pose)))
            saved[candidate.candidate_id]=copy.deepcopy(binding)
            stages=[dict(stage='PREPLAN_APPROACH',success=True,approach_ik=binding)]
            checks=[dict(code=name,status='PASS') for name in (
                'GENERATE_GRASPS','PREPLAN_APPROACH','PREPLAN_GRASP',
                'PREPLAN_CLOSE_GRIPPER','GRASP_GEOMETRY','ATTACH','PREPLAN_LIFT')[:7 if index==1 else 5]]
        code='CANDIDATE_SLICE_EXHAUSTED' if retryable else 'PREPLAN_GRASP_FAILED'
        checks.append(dict(code=failed,status='FAIL',failed_stage=failed,
                           failure_kind='budget' if retryable else 'collision'))
        return NS(success=False,candidate_id=candidate.candidate_id,reason_code=code,
                  reason='recorded discovery outcome',checks=checks,stages=stages,
                  extraction_attempts=[],cycle=None)

    monkeypatch.setattr(full_cycle_preplanner,'preplan_full_cycle',observed_discovery_then_real_retry)
    cycle=runtime.plan_authored_cycle(**kwargs)
    assert discovered==[(target['id'],f'finger_pinch_basic::{index:03d}')
                       for target in kwargs['targets'] for index in range(4)][:discovery_count]
    assert sum(durations)==pytest.approx(480.151)
    attempts=kwargs['summary']['candidate_attempts']
    assert len(attempts)==discovery_count+1
    assert all(a['candidate_wall_budget']==60. and a['planning_attempts']==1
               and a['segment_planning_time']==.75 for a in attempts[:discovery_count])
    retry=attempts[-1]
    assert (retry['object_id'],retry['candidate_id'])==discovered[1]
    assert retry['candidate_wall_budget']==retry_budget and retry['planning_attempts']==3
    assert retry['segment_planning_time']==3. and retry['candidate_wall_seconds']==32.
    assert clock[0]<1900. and kwargs['deadline']==1900.
    assert kwargs['summary']['candidate_search']['retry_pass_used'] is True
    phase=kwargs['summary']['candidate_search']['phases'][0]
    assert phase['discovered_candidates']==discovery_count and phase['retryable_candidates']==2
    assert phase['discovery_stop_reason']=='RETRY_BUDGET_RESERVED'
    discovery_elapsed=sum(durations[:discovery_count])+60.*max(0,discovery_count-len(durations))
    assert phase['remaining_wall_seconds']==pytest.approx(900.-discovery_elapsed)
    assert phase['retry_reserved_wall_seconds']==2*retry_budget
    assert phase['retry_priority'][0]['progress_passes']==7
    assert len(kwargs['summary']['candidate_search']['phases'])==1
    assert settings==[('retry:finger_pinch_basic',3,3.)]*9
    assert cycle['full_cycle_prevalidated']
    assert [step['stage'] for step in cycle['steps']]==EXPECTED
    assert any(check['code']=='DESTINATION_CONTAINMENT' and check['status']=='PASS'
               for check in retry['checks'])
    assert validity==pytest.approx([.01005,.0201])
    assert cycle['steps'][-1]['after'].robot_state==initial.robot_state
    assert cycle['steps'][-1]['after'].allowed_collision_matrix==initial.allowed_collision_matrix
    assert kwargs['initial_scene']==initial


def test_retry_reservation_without_retryable_candidate_keeps_discovery_order(monkeypatch):
    import full_cycle_preplanner
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'PREFERRED')
    kwargs.update(deadline=clock[0]+300.,discovery_candidate_budget=60.,reserve_retry_budget=True)
    calls=[]
    real_preplan=full_cycle_preplanner.preplan_full_cycle
    def deterministic_failures_then_complete(**arguments):
        candidate=arguments['candidate']
        calls.append(candidate.candidate_id)
        if candidate.candidate_id=='top_2f::007':
            return real_preplan(**arguments)
        clock[0]+=1.
        return NS(success=False,candidate_id=candidate.candidate_id,
                  reason_code='PREPLAN_APPROACH_FAILED',reason='IK failed',
                  checks=[dict(code='PREPLAN_APPROACH',status='FAIL',
                               failure_kind='planning',moveit_code=-31)],
                  stages=[],extraction_attempts=[],cycle=None)
    monkeypatch.setattr(full_cycle_preplanner,'preplan_full_cycle',deterministic_failures_then_complete)
    cycle=runtime.plan_authored_cycle(**kwargs)
    assert calls==[f'top_2f::{index:03d}' for index in range(8)]
    assert cycle['candidate'].candidate_id=='top_2f::007'
    assert [step['stage'] for step in cycle['steps']]==EXPECTED
    assert kwargs['summary']['candidate_search']['retry_pass_used'] is False
    assert 'discovery_stop_reason' not in kwargs['summary']['candidate_search']['phases'][0]
    assert settings==[('discovery:top_2f',1,.75)]*9


def test_retry_reservation_does_not_rename_true_global_expiry(monkeypatch):
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'AUTO')
    kwargs.update(deadline=clock[0]+23.,discovery_candidate_budget=60.,reserve_retry_budget=True)
    with pytest.raises(RuntimeError):
        runtime.plan_authored_cycle(**kwargs)
    attempt,=kwargs['summary']['candidate_attempts']
    assert attempt['reason_code']=='SEARCH_BUDGET_EXHAUSTED'
    assert kwargs['summary']['task_intent_resolution']['readiness']['primary_code']=='SEARCH_BUDGET_EXHAUSTED'
    assert kwargs['summary']['candidate_search']['retry_pass_used'] is False
    assert 'discovery_stop_reason' not in kwargs['summary']['candidate_search']['phases'][0]
    assert [goal[0] for goal in goals]==['PREPLAN_APPROACH']


def test_retry_reservation_leaves_resolved_revalidation_unchanged(monkeypatch):
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'AUTO')
    kwargs.update(deadline=clock[0]+1000.,discovery_candidate_budget=60.)
    selected=runtime.plan_authored_cycle(**kwargs)
    resolved=copy.deepcopy(kwargs['summary']['task_intent_resolution'])
    goals.clear();validity.clear();settings.clear()
    kwargs.update(deadline=clock[0]+100.,discovery_candidate_budget=.75,
                  reserve_retry_budget=True,resolved=resolved,summary={'candidate_attempts':[]})
    cycle=runtime.plan_authored_cycle(**kwargs)
    assert cycle['candidate']==selected['candidate']
    assert [step['stage'] for step in cycle['steps']]==EXPECTED
    assert settings==[('revalidate',3,3.)]*9
    attempt,=kwargs['summary']['candidate_attempts']
    assert attempt['candidate_wall_budget'] is None
    assert kwargs['summary']['candidate_search']['mode']=='resolved_revalidation'
    assert kwargs['summary']['resolution_sha256']==resolved['resolution_sha256']


@pytest.mark.parametrize('fallback_success',[True,False])
def test_reserved_discovery_reaches_fallback_and_reports_actual_failures(monkeypatch,fallback_success):
    import full_cycle_preplanner
    kwargs,clock,_,_,_=timed_authored_fixture(monkeypatch,'PREFERRED')
    kwargs.update(deadline=clock[0]+300.,discovery_candidate_budget=60.,reserve_retry_budget=True)
    calls=[]
    def evaluate(**arguments):
        candidate=arguments['candidate'];phase=kwargs['contract']['_candidate_search_pass']
        calls.append((phase,candidate.candidate_id));clock[0]+=10.
        if candidate.strategy_ref!='top_2f' and fallback_success:
            cycle=dict(object_id=arguments['observation']['id'],candidate=copy.deepcopy(candidate),
                       steps=[{'metadata':{}}],full_cycle_prevalidated=True)
            return NS(success=True,candidate_id=candidate.candidate_id,reason_code=None,reason=None,
                      checks=[dict(code='CANDIDATE_READY',status='PASS')],stages=[],extraction_attempts=[],cycle=cycle)
        code='CANDIDATE_SLICE_EXHAUSTED' if candidate.strategy_ref=='top_2f' else 'GRASP_GEOMETRY_INVALID'
        check=dict(code='PREPLAN_HOME',status='FAIL',reason_code=code,
                   candidate_id=candidate.candidate_id,failed_stage='PREPLAN_HOME',failure_kind='budget')
        return NS(success=False,candidate_id=candidate.candidate_id,reason_code=code,
                  reason='actual candidate rejection',checks=[check],stages=[dict(check,stage='PREPLAN_HOME',success=False)],
                  extraction_attempts=[],cycle=None)
    monkeypatch.setattr(full_cycle_preplanner,'preplan_full_cycle',evaluate)
    if fallback_success:
        cycle=runtime.plan_authored_cycle(**kwargs)
        assert cycle['candidate'].candidate_id=='side_grip_basic::000'
    else:
        with pytest.raises(RuntimeError,match='GRASP_GEOMETRY_INVALID'):
            runtime.plan_authored_cycle(**kwargs)
    assert calls[:11]==[('discovery:top_2f','top_2f::000'),
                       ('discovery:top_2f','top_2f::001'),
                       ('retry:top_2f','top_2f::000'),
                       ('retry:top_2f','top_2f::001')]+[
                       ('discovery:top_2f',f'top_2f::{i:03d}') for i in range(2,8)]+[
                       ('discovery:side_grip_basic','side_grip_basic::000')]
    search=kwargs['summary']['candidate_search']
    assert search['phases'][0]['discovery_stop_reason']=='RETRY_BUDGET_RESERVED'
    assert search['phases'][0]['retries'][0]['result_code']=='CANDIDATE_SLICE_EXHAUSTED'
    assert search['phases'][0]['retryable_candidates']==8
    assert search['phases'][0]['resumed_discovered_candidates']==2
    assert search['phases'][0]['equivalent_candidates_evaluated']==4
    assert search['phases'][1]['discovered_candidates']>0
    result=kwargs['summary']['task_intent_resolution']
    actual_ids={cid for _,cid in calls}
    assert all(a['candidate_id'] in actual_ids for a in result['grasp_resolution']['attempts'])
    assert all(a['reason_code']!='STRATEGY_PHASE_COMPLETE' for a in result['grasp_resolution']['attempts'])
    if not fallback_success:
        assert result['readiness']['primary_code']=='GRASP_GEOMETRY_INVALID'
        assert result['readiness']['checks'][0]['candidate_id']==calls[-1][1]


@pytest.mark.parametrize('retry_budget,global_remaining,expected',[
    (120.,900.,'CANDIDATE_SLICE_EXHAUSTED'),
    (180.,900.,None),
    (180.,120.5,'SEARCH_BUDGET_EXHAUSTED'),
])
def test_measured_eight_stage_cost_needs_retry_time_for_home(
        monkeypatch,retry_budget,global_remaining,expected):
    """Replay measured costs as unit inputs, keeping real cycle/scene checks.

    In 2026-10-01 02-cancel, eight successful stages plus qualification/service
    overhead took 120.906752 seconds; HOME was never sent under the 120s slice.
    The final 5s HOME here is a bounded unit response, not runtime evidence.
    """
    from dataclasses import replace
    import full_cycle_preplanner
    from tests.test_task_intent_resolver import valid_intent,environment,cell
    kwargs,_,goals,validity=fixture()
    observation=dict(kwargs.pop('observation'),confidence=.9,class_id='bottle',shape='BOX')
    kwargs.pop('candidate');kwargs.pop('destination')
    env=environment();env['task_zones'][1]['placement_local']['dimensions'][2]=.2
    env['task_zones'][1]['dimensions'][2]=.2
    initial=copy.deepcopy(kwargs['initial_scene'])
    clock=[1000.];fake_time=NS(monotonic=lambda:clock[0],time=time.time)
    monkeypatch.setattr(runtime,'time',fake_time)
    monkeypatch.setattr(full_cycle_preplanner,'time',fake_time)
    stages=[name for name in EXPECTED if name not in ('ATTACH','DETACH')]
    costs=dict(zip(stages,[20.005274296,.156789064,.269865990,49.818341255,
                          36.052384377,1.773729563,1.244194508,.878094196,5.]))
    overhead=120.906751719-sum(costs[name] for name in stages[:-1])
    plan=kwargs['operations'].plan_segment
    def measured_boundary(view,name,goal,group=None,straight=False):
        # Aggregate uninstrumented local work into the final completed boundary;
        # the observed deadline failure was before HOME, after RETREAT returned.
        clock[0]+=costs[name]+(overhead if name=='PREPLAN_RETREAT' else 0.)
        return plan(view,name,goal,group,straight)
    kwargs['operations']=replace(kwargs['operations'],plan_segment=measured_boundary)
    kwargs.update(intent=valid_intent('PREFERRED'),environment=env,cell=cell(),targets=[observation],
                  summary={'candidate_attempts':[]},deadline=1900.,retry_candidate_budget=retry_budget,
                  discovery_candidate_budget=60.,reserve_retry_budget=True)
    real_preplan=full_cycle_preplanner.preplan_full_cycle
    discovery_started=[False]
    def discover_then_retry(**arguments):
        candidate=arguments['candidate']
        if kwargs['contract']['_candidate_search_pass'].startswith('retry:'):
            return real_preplan(**arguments)
        if not discovery_started[0]:
            clock[0]+=900.-global_remaining;discovery_started[0]=True
        return NS(success=False,candidate_id=candidate.candidate_id,
                  reason_code='CANDIDATE_SLICE_EXHAUSTED',reason='discovery slice exhausted',
                  checks=[dict(code='PREPLAN_HOME',status='FAIL',failure_kind='budget')],
                  stages=[],extraction_attempts=[],cycle=None)
    monkeypatch.setattr(full_cycle_preplanner,'preplan_full_cycle',discover_then_retry)
    if expected:
        with pytest.raises(RuntimeError):runtime.plan_authored_cycle(**kwargs)
    else:
        cycle=runtime.plan_authored_cycle(**kwargs)
        assert cycle['full_cycle_prevalidated']
        assert [step['stage'] for step in cycle['steps']]==EXPECTED
        assert cycle['steps'][-1]['after'].robot_state==initial.robot_state
        assert cycle['steps'][-1]['after'].allowed_collision_matrix==initial.allowed_collision_matrix
    retries=[a for a in kwargs['summary']['candidate_attempts'] if a['search_pass'].startswith('retry:')]
    assert retries[0]['reason_code']==expected
    assert retries[0]['candidate_wall_budget']==retry_budget
    assert retries[0]['candidate_wall_seconds']==pytest.approx(120.906751719+(0. if expected else 5.))
    assert retries[0]['stages'][-1]['stage']=='PREPLAN_HOME'
    assert retries[0]['stages'][-1]['success'] is (expected is None)
    assert kwargs['deadline']==1900. and kwargs['initial_scene']==initial
    if expected=='SEARCH_BUDGET_EXHAUSTED':
        assert len(retries)==1
        assert kwargs['summary']['task_intent_resolution']['readiness']['primary_code']==expected
    if not expected:
        assert len(retries)==1 and len(goals)==9
        assert validity==pytest.approx([.01005,.0201])
        assert any(c['code']=='DESTINATION_CONTAINMENT' and c['status']=='PASS' for c in retries[0]['checks'])


def test_authored_retry_prioritizes_deepest_discovery_progress(monkeypatch):
    """A later candidate that reached lift is retried before shallow timeouts."""
    from types import SimpleNamespace
    import full_cycle_preplanner
    from tests.test_task_intent_resolver import valid_intent, environment, cell

    kwargs, _, _, _ = fixture()
    observation = dict(kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate'); kwargs.pop('destination')
    intent = valid_intent('AUTO')
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2
    counts = {}

    def fake_preplan_full_cycle(*, observation, candidate, **unused):
        cid = candidate.candidate_id
        counts[cid] = counts.get(cid, 0) + 1
        if counts[cid] == 1:
            if cid == 'top_2f::003':
                checks = [
                    {'code':'PREPLAN_APPROACH','status':'PASS'},
                    {'code':'PREPLAN_GRASP','status':'PASS'},
                    {'code':'PREPLAN_CLOSE_GRIPPER','status':'PASS'},
                    {'code':'ATTACH','status':'PASS'},
                    {'code':'PREPLAN_LIFT','status':'FAIL','failed_stage':'PREPLAN_LIFT',
                     'failure_kind':'planning','moveit_code':-6},
                ]
                code, reason = 'PREPLAN_LIFT_FAILED', 'timed out after physical grasp'
            else:
                checks = [
                    {'code':'PREPLAN_APPROACH','status':'FAIL','failed_stage':'PREPLAN_APPROACH',
                     'failure_kind':'planning','moveit_code':-6},
                ]
                code, reason = 'PREPLAN_APPROACH_FAILED', 'pregrasp timed out'
            return SimpleNamespace(extraction_attempts=[],
                success=False, candidate_id=cid, reason_code=code, reason=reason,
                checks=checks, stages=[], cycle=None)
        cycle = {
            'object_id': observation['id'], 'candidate': copy.deepcopy(candidate),
            'steps':[{'metadata':{}}], 'full_cycle_prevalidated':True}
        return SimpleNamespace(extraction_attempts=[],
            success=True, candidate_id=cid, reason_code=None, reason=None,
            checks=[{'code':'CANDIDATE_READY','status':'PASS'}],
            stages=[], cycle=cycle)

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle', fake_preplan_full_cycle)
    summary = {'candidate_attempts': []}
    cycle = runtime.plan_authored_cycle(
        intent=intent, environment=env, cell=cell(), targets=[observation],
        summary=summary, planning_time=3.0, **kwargs)

    retries = [a for a in summary['candidate_attempts']
               if a['search_pass'].startswith('retry:')]
    assert len(retries) == 1
    assert retries[0]['candidate_id'] == 'top_2f::003'
    assert cycle['candidate'].candidate_id == 'top_2f::003'
    search = summary['candidate_search']
    assert search['mode'] == 'strategy_phased_progress_beam'
    assert search['phases'][0]['retry_priority'][0]['candidate_id'] == 'top_2f::003'
    assert search['phases'][0]['retry_priority'][0]['progress_passes'] == 4


def test_retry_reuses_discovery_proven_approach_ik_branch(monkeypatch):
    """Deep discovery must not throw away an already-proven approach branch."""
    from types import SimpleNamespace
    import full_cycle_preplanner
    from tests.test_task_intent_resolver import valid_intent, environment, cell

    kwargs, _, _, _ = fixture()
    observation = dict(
        kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate')
    kwargs.pop('destination')
    intent = valid_intent('AUTO')
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2

    binding = {
        'schema': 'workcell_approach_ik/v1',
        'robot_model_sha256': 'fixture',
        'planning_group': 'arm',
        'tool_link': 'tool',
        'frame_id': 'world',
        'target_pose': [0., 0., 0., 0., 0., 0., 1.],
        'joint_positions': {'x': 0.1},
    }
    calls = {}

    def fake_preplan_full_cycle(*, observation, candidate, contract, **unused):
        key = candidate.candidate_id
        calls[key] = calls.get(key, 0) + 1
        if calls[key] == 1:
            if key == 'top_2f::003':
                checks = [
                    {'code':'PREPLAN_APPROACH','status':'PASS'},
                    {'code':'PREPLAN_GRASP','status':'PASS'},
                    {'code':'PREPLAN_CLOSE_GRIPPER','status':'PASS'},
                    {'code':'ATTACH','status':'PASS'},
                    {'code':'PREPLAN_LIFT','status':'PASS'},
                    {'code':'PREPLAN_TRANSFER','status':'FAIL',
                     'failed_stage':'PREPLAN_TRANSFER',
                     'failure_kind':'budget'},
                ]
                stages = [
                    {'stage':'PREPLAN_APPROACH','success':True,
                     'moveit_code':1,'approach_ik':copy.deepcopy(binding)},
                    {'stage':'PREPLAN_GRASP','success':True,'moveit_code':1},
                    {'stage':'PREPLAN_CLOSE_GRIPPER','success':True,'moveit_code':1},
                    {'stage':'PREPLAN_LIFT','success':True,'moveit_code':1},
                    {'stage':'PREPLAN_TRANSFER','success':False,
                     'reason_code':'CANDIDATE_SLICE_EXHAUSTED'},
                ]
                return SimpleNamespace(extraction_attempts=[],
                    success=False, candidate_id=key,
                    reason_code='CANDIDATE_SLICE_EXHAUSTED',
                    reason='candidate wall-clock slice exhausted',
                    checks=checks, stages=stages, cycle=None)
            return SimpleNamespace(extraction_attempts=[],
                success=False, candidate_id=key,
                reason_code='PREPLAN_APPROACH_FAILED', reason='blocked',
                checks=[{'code':'PREPLAN_APPROACH','status':'FAIL',
                         'failed_stage':'PREPLAN_APPROACH',
                         'failure_kind':'planning','moveit_code':-2}],
                stages=[{'stage':'PREPLAN_APPROACH','success':False}],
                cycle=None)

        assert key == 'top_2f::003'
        assert contract.get('approach_ik') == binding
        cycle = {
            'object_id': observation['id'],
            'candidate': copy.deepcopy(candidate),
            'steps': [
                {'metadata': {'stage':'PREPLAN_APPROACH',
                              'approach_ik':copy.deepcopy(binding)}}
            ],
            'full_cycle_prevalidated': True,
        }
        return SimpleNamespace(extraction_attempts=[],
            success=True, candidate_id=key, reason_code=None, reason=None,
            checks=[{'code':'CANDIDATE_READY','status':'PASS'}],
            stages=[], cycle=cycle)

    monkeypatch.setattr(
        full_cycle_preplanner, 'preplan_full_cycle', fake_preplan_full_cycle)

    summary = {'candidate_attempts': []}
    cycle = runtime.plan_authored_cycle(
        intent=intent, environment=env, cell=cell(), targets=[observation],
        summary=summary, planning_time=3.0, **kwargs)

    assert cycle['candidate'].candidate_id == 'top_2f::003'
    retries = [
        item for item in summary['candidate_attempts']
        if item['search_pass'].startswith('retry:')
    ]
    assert len(retries) == 1
    assert retries[0]['candidate_id'] == 'top_2f::003'


def test_preferred_strategy_retries_before_fallback_discovery(monkeypatch):
    """PREFERRED must spend its retry beam before evaluating fallback strategy candidates."""
    from types import SimpleNamespace
    import full_cycle_preplanner
    from tests.test_task_intent_resolver import valid_intent, environment, cell

    kwargs, _, _, _ = fixture()
    observation = dict(kwargs.pop('observation'), confidence=.9, class_id='bottle', shape='BOX')
    kwargs.pop('candidate'); kwargs.pop('destination')
    intent = valid_intent('PREFERRED')
    env = environment()
    env['task_zones'][1]['placement_local']['dimensions'][2] = .2
    env['task_zones'][1]['dimensions'][2] = .2
    calls = []

    def fake_preplan_full_cycle(*, observation, candidate, **unused):
        calls.append(candidate.strategy_ref)
        if candidate.strategy_ref == 'top_2f':
            return SimpleNamespace(extraction_attempts=[],
                success=False, candidate_id=candidate.candidate_id,
                reason_code='PREPLAN_APPROACH_FAILED', reason='timeout',
                checks=[{'code':'PREPLAN_APPROACH','status':'FAIL',
                         'failed_stage':'PREPLAN_APPROACH',
                         'failure_kind':'planning','moveit_code':-6}],
                stages=[], cycle=None)
        cycle = {
            'object_id': observation['id'], 'candidate': copy.deepcopy(candidate),
            'steps':[{'metadata':{}}], 'full_cycle_prevalidated':True}
        return SimpleNamespace(extraction_attempts=[],
            success=True, candidate_id=candidate.candidate_id,
            reason_code=None, reason=None,
            checks=[{'code':'CANDIDATE_READY','status':'PASS'}],
            stages=[], cycle=cycle)

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle', fake_preplan_full_cycle)
    summary = {'candidate_attempts': []}
    cycle = runtime.plan_authored_cycle(
        intent=intent, environment=env, cell=cell(), targets=[observation],
        summary=summary, planning_time=3.0, **kwargs)

    search = summary['candidate_search']
    assert search['strategy_phases'][:2] == ['top_2f', 'side_grip_basic']
    assert search['phases'][0]['discovered_candidates'] == 8
    assert len(search['phases'][0]['retries']) == 3
    # Only after three bounded retries of the preferred top strategy may the
    # fallback side strategy be evaluated, where this fixture succeeds.
    assert calls[:11] == ['top_2f'] * 11
    assert calls[11] == 'side_grip_basic'
    assert cycle['candidate'].strategy_ref == 'side_grip_basic'


def test_authored_destination_orientation_is_planned_with_actual_grasp_transform():
    from full_cycle_preplanner import preplan_full_cycle
    from grasp_strategy_candidates import generate_strategy_candidates
    from tests.test_task_intent_resolver import valid_intent
    kwargs, trace, goals, _ = fixture()
    q = runtime._PLANNER.quaternion_from_rpy([0.,0.,.23])
    kwargs['observation']['pose'][3:] = q
    obj = kwargs['initial_scene'].world.collision_objects[0]
    obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w = q
    kwargs['candidate'] = generate_strategy_candidates('top_2f',kwargs['observation'],{'approach_distance_m':.12})[3]
    kwargs['contract']['task_intent'] = valid_intent('AUTO')
    result = preplan_full_cycle(**kwargs)
    assert result.success, result.reason
    placed = result.cycle['steps'][-1]['after'].world.collision_objects[-1]
    achieved = runtime.pose_values(placed.pose)
    assert achieved[:3] == pytest.approx(kwargs['destination']['pose_xyz'])
    assert abs(achieved[-1]) == pytest.approx(1.)
    assert achieved[3:6] == pytest.approx([0.,0.,0.],abs=1e-9)
    assert [g[0] for g in goals] == [s for s in EXPECTED if s.startswith('PREPLAN_')]
    assert result.cycle['steps'][4]['stage'] == 'PREPLAN_LIFT'


def test_preplanner_uses_bound_resolution_freshness_reference(monkeypatch):
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, _ = fixture()
    captured = kwargs['observation']['timestamp']
    kwargs['contract']['max_age_seconds'] = 5.0
    kwargs['contract']['observation_reference_time'] = captured + 1.0
    monkeypatch.setattr(time, 'time', lambda: captured + 1000.0)
    result = preplan_full_cycle(**kwargs)
    assert result.success, result.reason


def test_transfer_seed_is_forwarded_only_to_fresh_transfer_planning():
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, _=fixture()
    seed={'schema':'workcell_transfer_ik_seed/v1','joint_positions':{'arm':.5}}
    kwargs['contract']['transfer_ik_seed']=copy.deepcopy(seed)
    original=kwargs['operations'].plan_segment
    calls=[]
    def segment(view,name,goal,group=None,straight=False,**options):
        calls.append((name,copy.deepcopy(options)))
        return original(view,name,goal,group,straight)
    kwargs['operations']=replace(kwargs['operations'],plan_segment=segment)
    result=preplan_full_cycle(**kwargs)
    assert result.success,result.reason
    assert next(options for name,options in calls if name=='PREPLAN_TRANSFER')=={'ik_seed':seed}
    assert all(not options for name,options in calls if name!='PREPLAN_TRANSFER')
    assert kwargs['contract']['transfer_ik_seed']==seed


def extraction_fixture(fail_stage='PREPLAN_LIFT', fail_all=False):
    from dataclasses import replace
    kwargs, trace, goals, validity = fixture()
    base=kwargs['operations']; seen=[]; active=[None]
    variants=[dict(schema='workcell_extraction_intent/v1',variant_id=name,
        object_id=kwargs['observation']['id'],candidate_id=kwargs['candidate'].candidate_id,
        offset_xyz_m=[x,0.,kwargs['contract']['retreat_distance_m']])
        for name,x in [('vertical',0.),('away',.002)]]
    def segment(view,name,goal,group=None,straight=False,**options):
        if name=='PREPLAN_LIFT':
            active[0]=options['extraction_intent']['variant_id']
            seen.append((active[0],copy.deepcopy(view)))
        if name==fail_stage and (fail_all or active[0]=='vertical'):
            raise RuntimeError('variant collision')
        return base.plan_segment(view,name,goal,group,straight)
    kwargs['operations']=replace(base,plan_segment=segment,
        extraction_candidates=lambda view,observation,candidate,retreat:copy.deepcopy(variants))
    return kwargs,seen,variants


@pytest.mark.parametrize('failed_stage',['PREPLAN_LIFT','PREPLAN_TRANSFER','PREPLAN_HOME'])
def test_extraction_variants_require_entire_suffix_and_restore_checkpoint(failed_stage):
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,seen,variants=extraction_fixture(failed_stage)
    result=preplan_full_cycle(**kwargs)
    assert result.success,result.reason
    assert [x[0] for x in seen]==['vertical','away']
    assert seen[0][1]==seen[1][1]
    assert result.cycle['extraction_intent']==variants[1]
    assert [x['stage'] for x in result.cycle['steps']]==EXPECTED
    assert all(c['status']=='PASS' for c in result.checks)
    assert result.extraction_attempts[0]['failed_stage']==failed_stage
    assert result.extraction_attempts[1]['success'] is True


def test_all_extraction_variants_fail_with_explicit_reason():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,seen,_=extraction_fixture(fail_all=True)
    result=preplan_full_cycle(**kwargs)
    assert not result.success and result.cycle is None
    assert result.reason_code=='NO_VALID_EXTRACTION'
    assert len(result.extraction_attempts)==2


def test_bound_extraction_revalidation_never_substitutes_variant():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,seen,variants=extraction_fixture()
    kwargs['contract']['extraction_intent']=copy.deepcopy(variants[0])
    result=preplan_full_cycle(**kwargs)
    assert not result.success and result.reason_code=='NO_VALID_EXTRACTION'
    assert [x[0] for x in seen]==['vertical']
    kwargs['contract']['extraction_intent']['offset_xyz_m'][0]=.001
    seen.clear();result=preplan_full_cycle(**kwargs)
    assert not result.success and result.reason_code=='EXTRACTION_INTENT_CHANGED'
    assert not seen


def test_unbound_transfer_seed_cannot_cross_extraction_variants():
    from full_cycle_preplanner import preplan_full_cycle
    from dataclasses import replace
    kwargs,_,_=extraction_fixture('PREPLAN_TRANSFER')
    base=kwargs['operations']; received=[]
    def segment(*args,**options):
        if args[1]=='PREPLAN_TRANSFER':received.append(options.get('ik_seed'))
        return base.plan_segment(*args,**options)
    kwargs['operations']=replace(base,plan_segment=segment)
    kwargs['contract']['transfer_ik_seed']={'unbound':'seed'}
    result=preplan_full_cycle(**kwargs)
    assert result.success,result.reason
    assert received==[None,None]


def test_extraction_exhaustion_preserves_later_phase_failure():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,_,_=extraction_fixture('PREPLAN_TRANSFER',fail_all=True)
    result=preplan_full_cycle(**kwargs)
    assert result.reason_code=='PREPLAN_TRANSFER_FAILED'
    assert len(result.extraction_attempts)==2


def test_extraction_budget_exhaustion_does_not_start_another_variant():
    from full_cycle_preplanner import preplan_full_cycle,CandidateBudgetExhausted
    from dataclasses import replace
    kwargs,_,_=extraction_fixture();base=kwargs['operations'];seen=[]
    def segment(*args,**options):
        if args[1]=='PREPLAN_LIFT':
            seen.append(options['extraction_intent']['variant_id'])
            raise CandidateBudgetExhausted('shared slice exhausted')
        return base.plan_segment(*args,**options)
    kwargs['operations']=replace(base,plan_segment=segment)
    result=preplan_full_cycle(**kwargs)
    assert result.reason_code=='CANDIDATE_SLICE_EXHAUSTED'
    assert seen==['vertical']


def test_planning_lifecycle_failure_at_lift_escapes_without_another_variant():
    from dataclasses import replace
    from full_cycle_preplanner import PlanningLifecycleFailure, preplan_full_cycle
    kwargs, _, _ = extraction_fixture(fail_stage=None)
    base = kwargs['operations']
    failure = PlanningLifecycleFailure('planning goal has no confirmed terminal state')
    calls, variants = [], []
    initial = copy.deepcopy(kwargs['initial_scene'])
    def segment(*args, **options):
        calls.append(args[1])
        if args[1] == 'PREPLAN_LIFT':
            variants.append(options['extraction_intent']['variant_id'])
            raise failure
        return base.plan_segment(*args, **options)
    kwargs['operations'] = replace(base, plan_segment=segment)
    with pytest.raises(PlanningLifecycleFailure) as caught:
        preplan_full_cycle(**kwargs)
    assert caught.value is failure
    assert calls == ['PREPLAN_APPROACH', 'PREPLAN_GRASP', 'PREPLAN_CLOSE_GRIPPER', 'PREPLAN_LIFT']
    assert variants == ['vertical']
    assert kwargs['initial_scene'] == initial


def test_bound_transfer_seed_is_forwarded_only_with_matching_extraction():
    from full_cycle_preplanner import preplan_full_cycle
    from dataclasses import replace
    kwargs,_,variants=extraction_fixture();base=kwargs['operations'];received=[]
    kwargs['contract'].update(extraction_intent=copy.deepcopy(variants[1]),transfer_ik_seed={'bound':'seed'})
    def segment(*args,**options):
        if args[1]=='PREPLAN_TRANSFER':received.append(options.pop('ik_seed',None))
        return base.plan_segment(*args,**options)
    kwargs['operations']=replace(base,plan_segment=segment)
    result=preplan_full_cycle(**kwargs)
    assert result.success,result.reason
    assert received==[{'bound':'seed'}]


def test_selected_extraction_has_explicit_feasibility_check_without_extra_motion():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,_,variants=extraction_fixture()
    result=preplan_full_cycle(**kwargs)
    evidence=next(c for c in result.checks if c['code']=='INITIAL_PILE_EXTRACTION')
    assert evidence['status']=='PASS'
    assert evidence['extraction_intent']==variants[1]
    assert [s['stage'] for s in result.cycle['steps']]==EXPECTED


@pytest.mark.parametrize('offset',[[.003,0.,.15],[float('nan'),0.,.15],[0.,0.,.14]])
def test_invalid_extraction_offset_fails_before_lift(offset):
    from full_cycle_preplanner import preplan_full_cycle
    from dataclasses import replace
    kwargs,seen,variants=extraction_fixture()
    variants[0]['offset_xyz_m']=offset
    kwargs['operations']=replace(kwargs['operations'],extraction_candidates=lambda *args:variants)
    result=preplan_full_cycle(**kwargs)
    assert not result.success and result.reason_code=='EXTRACTION_INTENT_INVALID'
    assert not seen


@pytest.mark.parametrize('all_fail',[False,True])
def test_real_extraction_failure_preserves_details_and_safe_variant_fallback(all_fail):
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    from pile_extraction import ExtractionFailure
    kwargs,seen,variants=extraction_fixture(fail_stage=None)
    base=kwargs['operations']; attempted=[]
    def segment(view,name,goal,group=None,straight=False,**options):
        if name=='PREPLAN_LIFT':
            variant=options['extraction_intent']['variant_id'];attempted.append(variant)
            if all_fail or variant=='vertical':
                raise ExtractionFailure('EXTRACTION_CLEARANCE_INSUFFICIENT',
                                        neighbor='fixture-neighbor',separation_m=.000002)
        return base.plan_segment(view,name,goal,group,straight,**options)
    kwargs['operations']=replace(base,plan_segment=segment)
    result=preplan_full_cycle(**kwargs)
    assert attempted==['vertical','away']
    rejected=result.extraction_attempts[0]
    assert rejected['failed_stage']=='PREPLAN_LIFT'
    assert rejected['failure_kind']=='extraction'
    assert rejected['extraction_reason_code']=='EXTRACTION_CLEARANCE_INSUFFICIENT'
    assert rejected['neighbor']=='fixture-neighbor'
    assert rejected['separation_m']==.000002
    assert 'TypeError' not in (result.reason or '')
    if all_fail:
        assert not result.success and result.cycle is None
        assert result.reason_code=='NO_VALID_EXTRACTION'
        assert result.checks[-1]['failure_kind']=='extraction'
        assert result.checks[-1]['extraction_reason_code']=='EXTRACTION_CLEARANCE_INSUFFICIENT'
    else:
        assert result.success,result.reason
        assert result.cycle['extraction_intent']==variants[1]
        assert [step['stage'] for step in result.cycle['steps']]==EXPECTED
        assert all(check['status']=='PASS' for check in result.checks)


def test_simulator_contact_membership_cannot_authorize_attachment_without_geometry():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,trace,_,_=closure_fixture(lambda i:['left_tip','right_tip'])
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code=='GRASP_GEOMETRY_FAILED'
    assert 'ATTACH' not in trace


def test_simulator_geometric_rejection_prevents_attachment_and_reports_margins():
    from dataclasses import replace
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,trace,_,_=closure_fixture(lambda i:['left_tip','right_tip'])
    kwargs['operations']=replace(kwargs['operations'],qualify_grasp=lambda **kw:dict(
        accepted=False,reason='synthetic edge contact: radius 0.00001 m',samples=[]))
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert 'ATTACH' not in trace
    assert '0.00001' in result.reason


def test_unsafe_closure_candidate_cannot_plan_approach_close_or_attachment():
    from dataclasses import replace
    from closure_clearance import ClosureClearanceFailure
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,trace,goals,_=closure_fixture(lambda q:['left_tip','right_tip'] if q>=.012 else [])
    def reject(**kw):
        raise ClosureClearanceFailure('CLOSURE_NEIGHBOR_CLEARANCE',neighbor='arbitrary',gap_m=.000007)
    kwargs['operations']=replace(kwargs['operations'],screen_closure=reject)
    result=preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code=='CLOSURE_CLEARANCE_REJECTED'
    assert goals==[]
    assert 'ATTACH' not in trace


def test_closure_sweep_receives_clearance_evidence_and_other_motions_do_not():
    from dataclasses import replace
    from test_grasp_capture import case
    from full_cycle_preplanner import preplan_full_cycle
    kwargs,_,_,_=closure_fixture(lambda q:['left_tip','right_tip'] if q>=.012 else [])
    qualify,data=case();original=kwargs['operations'].plan_segment;seen=[]
    evidence={'accepted':True,'neighbor_inflation_m':{'neighbor':.003}}
    def segment(*args,**options):
        seen.append((args[1],copy.deepcopy(options)))
        options.pop('closure_clearance',None)
        return original(*args,**options)
    kwargs['operations']=replace(kwargs['operations'],plan_segment=segment,
        screen_closure=lambda **kw:copy.deepcopy(evidence),qualify_grasp=lambda **kw:qualify(**data))
    result=preplan_full_cycle(**kwargs)
    assert result.success,result.reason
    assert [(name,opts['closure_clearance']) for name,opts in seen if 'closure_clearance' in opts]==[
        ('PREPLAN_CLOSE_GRIPPER',evidence)]


def test_unused_retry_slots_cannot_hide_later_feasible_catalog_candidate(monkeypatch):
    """One actual retry cannot reserve three slots and strand fresh candidates."""
    import full_cycle_preplanner
    kwargs,clock,goals,validity,settings=timed_authored_fixture(monkeypatch,'PREFERRED')
    grasp=kwargs['intent']['pick']['grasp']
    grasp.update(strategy_ref='finger_pinch_basic',approach={'axis':'tool_z','distance_m':.09})
    grasp['orientation']['mode']='tool_aligned'
    kwargs.update(deadline=clock[0]+700.,discovery_candidate_budget=60.,
                  retry_candidate_budget=180.,reserve_retry_budget=True)
    original=full_cycle_preplanner.preplan_full_cycle
    called=[]
    def first_stochastic_then_existing_safe_cycle(**arguments):
        candidate=arguments['candidate'];phase=kwargs['contract']['_candidate_search_pass']
        called.append((phase,candidate.candidate_id))
        if candidate.candidate_id=='finger_pinch_basic::001':
            return original(**arguments)
        if candidate.candidate_id=='finger_pinch_basic::000':
            clock[0]+=100. if phase.startswith('discovery:') else 1.
            kind,code,reason='planning',-6,'PREPLAN_APPROACH_FAILED'
        else:
            kind,code,reason='collision',-2,'PREPLAN_GRASP_FAILED'
        return NS(success=False,candidate_id=candidate.candidate_id,reason_code=reason,
            reason='controlled feasibility outcome',cycle=None,extraction_attempts=[],stages=[],
            checks=[dict(code='PREPLAN_APPROACH',status='FAIL',failure_kind=kind,moveit_code=code)])
    monkeypatch.setattr(full_cycle_preplanner,'preplan_full_cycle',first_stochastic_then_existing_safe_cycle)
    cycle=runtime.plan_authored_cycle(**kwargs)
    assert cycle['full_cycle_prevalidated']
    assert cycle['candidate'].candidate_id=='finger_pinch_basic::001'
    assert called==[('discovery:finger_pinch_basic','finger_pinch_basic::000'),
                    ('discovery:finger_pinch_basic','finger_pinch_basic::001')]
    assert clock[0]<kwargs['deadline']
    assert kwargs['summary']['candidate_search']['retry_candidate_wall_budget']==180.
    assert kwargs['summary']['candidate_search']['retry_pass_used'] is False


@pytest.mark.parametrize('retry_count', [1, 3])
@pytest.mark.parametrize('later_safe', [False, True])
def test_discovery_resumes_after_failed_retries_without_repeating_candidates(
        monkeypatch, retry_count, later_safe):
    """Unused retry time must reach fresh candidates through every normal gate."""
    import full_cycle_preplanner
    kwargs, clock, _, _, _ = timed_authored_fixture(monkeypatch, 'PREFERRED')
    grasp = kwargs['intent']['pick']['grasp']
    grasp.update(strategy_ref='finger_pinch_basic',
                 approach={'axis': 'tool_z', 'distance_m': .09})
    grasp['orientation']['mode'] = 'tool_aligned'
    kwargs.update(deadline=clock[0] + (300. if retry_count == 1 else 800.),
                  discovery_candidate_budget=60., retry_candidate_budget=180.,
                  reserve_retry_budget=True)
    original = full_cycle_preplanner.preplan_full_cycle
    calls = []

    def controlled_failure_then_real_cycle(**arguments):
        candidate = arguments['candidate']
        phase = kwargs['contract']['_candidate_search_pass']
        calls.append((phase, candidate.candidate_id))
        index = int(candidate.candidate_id.rsplit('::', 1)[1])
        if (candidate.strategy_ref == 'finger_pinch_basic' and
                index < retry_count):
            clock[0] += 100. if phase.startswith('discovery:') else 1.
            kind, code, reason = 'planning', -6, 'PREPLAN_APPROACH_FAILED'
        elif (later_safe and candidate.strategy_ref == 'finger_pinch_basic'
              and index == retry_count):
            return original(**arguments)
        else:
            kind, code, reason = 'collision', -2, 'PREPLAN_GRASP_FAILED'
        return NS(success=False, candidate_id=candidate.candidate_id,
                  reason_code=reason, reason='controlled feasibility outcome',
                  cycle=None, extraction_attempts=[], stages=[], checks=[dict(
                      code='PREPLAN_APPROACH', status='FAIL', failure_kind=kind,
                      moveit_code=code)])

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle',
                        controlled_failure_then_real_cycle)
    if later_safe:
        cycle = runtime.plan_authored_cycle(**kwargs)
        assert cycle['full_cycle_prevalidated']
        assert cycle['candidate'].candidate_id == f'finger_pinch_basic::{retry_count:03d}'
    else:
        with pytest.raises(RuntimeError):
            runtime.plan_authored_cycle(**kwargs)
    expected = [('discovery:finger_pinch_basic', f'finger_pinch_basic::{i:03d}')
                for i in range(retry_count)]
    expected += [('retry:finger_pinch_basic', f'finger_pinch_basic::{i:03d}')
                 for i in range(retry_count)]
    expected += [('discovery:finger_pinch_basic',
                  f'finger_pinch_basic::{retry_count:03d}')]
    assert calls[:len(expected)] == expected
    assert len(calls) == len(set(calls))
    assert clock[0] < kwargs['deadline']
    phase = kwargs['summary']['candidate_search']['phases'][0]
    assert phase['resumed_discovered_candidates'] >= 1
    assert len(phase['retries']) == retry_count


def test_distinct_object_poses_precede_equivalent_catalogue_aliases(monkeypatch):
    """Repeated poses cannot consume the budget before another object's first pose."""
    import full_cycle_preplanner
    kwargs, clock, _, _, _ = timed_authored_fixture(monkeypatch, 'PREFERRED')
    second = kwargs['targets'][0]
    first = copy.deepcopy(second)
    first['id'] = 'a-prior-box'
    kwargs['targets'] = [first, second]
    obj = copy.deepcopy(kwargs['initial_scene'].world.collision_objects[0])
    obj.id = first['id']
    kwargs['initial_scene'].world.collision_objects.append(obj)
    kwargs.update(deadline=clock[0] + 700., discovery_candidate_budget=60.,
                  reserve_retry_budget=True)
    original = full_cycle_preplanner.preplan_full_cycle
    calls = []

    def slow_first_object_then_real_safe_cycle(**arguments):
        candidate = arguments['candidate']
        calls.append((arguments['observation']['id'], candidate.candidate_id))
        if arguments['observation']['id'] == second['id']:
            return original(**arguments)
        clock[0] += 100.
        expired = clock[0] >= kwargs['deadline']
        code = 'SEARCH_BUDGET_EXHAUSTED' if expired else 'PREPLAN_GRASP_FAILED'
        return NS(success=False, candidate_id=candidate.candidate_id,
                  reason_code=code, reason='controlled collision or expired budget',
                  cycle=None, extraction_attempts=[], stages=[], checks=[dict(
                      code='PREPLAN_GRASP', status='FAIL', failure_kind='budget' if expired else 'collision')])

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle',
                        slow_first_object_then_real_safe_cycle)
    cycle = runtime.plan_authored_cycle(**kwargs)
    assert cycle['object_id'] == second['id']
    assert cycle['full_cycle_prevalidated']
    assert calls == [(first['id'], f'top_2f::{i:03d}') for i in range(4)] + [
        (second['id'], 'top_2f::000')]
    assert clock[0] < kwargs['deadline']
    assert len(kwargs['summary']['candidate_search']['phases'][0][
        'deferred_equivalent_candidates']) == 4


def test_deferred_pose_alias_keeps_its_own_successful_cycle_validation(monkeypatch):
    import full_cycle_preplanner
    kwargs, clock, goals, _, _ = timed_authored_fixture(monkeypatch, 'PREFERRED')
    kwargs.update(deadline=clock[0] + 700., discovery_candidate_budget=60.,
                  reserve_retry_budget=True)
    original = full_cycle_preplanner.preplan_full_cycle
    calls = []

    def failures_then_real_alias(**arguments):
        candidate = arguments['candidate']
        calls.append(candidate.candidate_id)
        if candidate.candidate_id == 'top_2f::005':
            return original(**arguments)
        return NS(success=False, candidate_id=candidate.candidate_id,
                  reason_code='PREPLAN_GRASP_FAILED', reason='controlled collision',
                  cycle=None, extraction_attempts=[], stages=[], checks=[dict(
                      code='PREPLAN_GRASP', status='FAIL', failure_kind='collision')])

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle', failures_then_real_alias)
    cycle = runtime.plan_authored_cycle(**kwargs)
    assert calls == [f'top_2f::{i:03d}' for i in range(6)]
    assert cycle['candidate'].candidate_id == 'top_2f::005'
    assert cycle['full_cycle_prevalidated']
    assert len(goals) == 9
    phase = kwargs['summary']['candidate_search']['phases'][0]
    assert phase['equivalent_candidates_evaluated'] == 2
    assert phase['discovered_candidates'] == 6


def test_deferred_aliases_cannot_hide_a_later_objects_selected_retry(monkeypatch):
    import full_cycle_preplanner
    kwargs, clock, _, _, _ = timed_authored_fixture(monkeypatch, 'PREFERRED')
    second = kwargs['targets'][0]
    first = copy.deepcopy(second)
    first['id'] = 'a-prior-box'
    kwargs['targets'] = [first, second]
    obj = copy.deepcopy(kwargs['initial_scene'].world.collision_objects[0])
    obj.id = first['id']
    kwargs['initial_scene'].world.collision_objects.append(obj)
    kwargs.update(deadline=clock[0] + 1000., discovery_candidate_budget=60.,
                  reserve_retry_budget=True)
    original = full_cycle_preplanner.preplan_full_cycle
    calls = []

    def real_selected_retry(**arguments):
        candidate = arguments['candidate']
        phase = kwargs['contract']['_candidate_search_pass']
        calls.append((arguments['observation']['id'], candidate.candidate_id, phase))
        selected = (arguments['observation']['id'] == second['id'] and
                    candidate.candidate_id == 'top_2f::000')
        if selected and phase.startswith('retry:'):
            return original(**arguments)
        return NS(success=False, candidate_id=candidate.candidate_id,
                  reason_code='PREPLAN_APPROACH_FAILED' if selected else 'PREPLAN_GRASP_FAILED',
                  reason='controlled feasibility outcome', cycle=None,
                  extraction_attempts=[], stages=[], checks=[dict(
                      code='PREPLAN_APPROACH' if selected else 'PREPLAN_GRASP',
                      status='FAIL', failure_kind='planning' if selected else 'collision',
                      moveit_code=-6 if selected else -2)])

    monkeypatch.setattr(full_cycle_preplanner, 'preplan_full_cycle', real_selected_retry)
    cycle = runtime.plan_authored_cycle(**kwargs)
    assert cycle['object_id'] == second['id'] and cycle['full_cycle_prevalidated']
    retries = [a for a in kwargs['summary']['candidate_attempts']
               if a['search_pass'].startswith('retry:')]
    assert len(retries) == 1
    assert (retries[0]['object_id'], retries[0]['candidate_id']) == (second['id'], 'top_2f::000')
    assert calls[-1] == (second['id'], 'top_2f::000', 'retry:top_2f')
    assert len(kwargs['summary']['candidate_search']['phases'][0]['retries']) == 1


@pytest.mark.parametrize('authored,candidate_offset,accepted', [
    ([0., 0., -.004], [0., 0., -.004], True),     # honoured: the candidate carries the very offset
    ([0., 0., -.004], None, False),               # authored but not consumed by the candidate
    ([0., 0., -.003], [0., 0., -.004], False),    # different offset
])
def test_pinch_tcp_offset_is_accepted_only_when_the_candidate_honours_it(authored, candidate_offset, accepted):
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    from tests.test_task_intent_resolver import valid_intent
    kwargs, _, goals, _ = fixture()
    intent = valid_intent('EXACT')
    grasp = intent['pick']['grasp']
    grasp.update(strategy_ref='finger_pinch_basic', tcp_offset_xyz_m=authored)
    grasp['approach']['axis'] = 'tool_z'; grasp['orientation']['mode'] = 'tool_aligned'
    kwargs['contract']['task_intent'] = intent
    constraints = {'approach_distance_m': .07}
    if candidate_offset is not None:
        constraints['tcp_offset_xyz_m'] = candidate_offset
    kwargs['candidate'] = generate_strategy_candidates('finger_pinch_basic', kwargs['observation'], constraints)[0]
    result = preplan_full_cycle(**kwargs)
    if accepted:
        assert result.success, result.reason
    else:
        assert not result.success and 'nonzero authored TCP adjustment' in result.reason and goals == []


def test_pinch_tcp_rotation_offset_is_still_refused():
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    from tests.test_task_intent_resolver import valid_intent
    kwargs, _, goals, _ = fixture()
    intent = valid_intent('EXACT'); grasp = intent['pick']['grasp']
    grasp.update(strategy_ref='finger_pinch_basic', tcp_offset_rpy_rad=[0., 0., .1])
    grasp['approach']['axis'] = 'tool_z'; grasp['orientation']['mode'] = 'tool_aligned'
    kwargs['contract']['task_intent'] = intent
    kwargs['candidate'] = generate_strategy_candidates('finger_pinch_basic', kwargs['observation'], {'approach_distance_m': .07})[0]
    result = preplan_full_cycle(**kwargs)
    assert not result.success and 'nonzero authored TCP adjustment' in result.reason and goals == []
