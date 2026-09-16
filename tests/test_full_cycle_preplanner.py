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
    assert [g[0] for g in goals if g[3]] == ['PREPLAN_GRASP', 'PREPLAN_RETREAT']
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
