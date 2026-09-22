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
    def segment(view, name, goal, group=None, straight=False):
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
    assert [g[0] for g in goals if g[3]] == ['PREPLAN_GRASP', 'PREPLAN_LIFT', 'PREPLAN_RETREAT']
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


def test_missing_contact_never_passes_on_validity_alone():
    from full_cycle_preplanner import preplan_full_cycle
    kwargs, _, _, validity = fixture(contacts=False)
    result = preplan_full_cycle(**kwargs)
    assert not result.success
    assert result.reason_code == 'PREPLAN_CLOSE_GRIPPER_FAILED'
    assert 'no allowed fingertip contact' in result.reason
    assert len(validity) == 80


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
        'private_attachment', 'object_pose_after_motion', 'place_detachment_diff', 'stage'}


@pytest.mark.parametrize('change,reason', [
    ('expired', 'observation expired'), ('deadline', 'budget exhausted'),
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
            return SimpleNamespace(
                success=False, candidate_id=cid, reason_code=code, reason=reason,
                checks=checks, stages=[], cycle=None)
        cycle = {
            'object_id': observation['id'], 'candidate': copy.deepcopy(candidate),
            'steps':[{'metadata':{}}], 'full_cycle_prevalidated':True}
        return SimpleNamespace(
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
            return SimpleNamespace(
                success=False, candidate_id=candidate.candidate_id,
                reason_code='PREPLAN_APPROACH_FAILED', reason='timeout',
                checks=[{'code':'PREPLAN_APPROACH','status':'FAIL',
                         'failed_stage':'PREPLAN_APPROACH',
                         'failure_kind':'planning','moveit_code':-6}],
                stages=[], cycle=None)
        cycle = {
            'object_id': observation['id'], 'candidate': copy.deepcopy(candidate),
            'steps':[{'metadata':{}}], 'full_cycle_prevalidated':True}
        return SimpleNamespace(
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
