#!/usr/bin/env python3
"""One plan-only full-cycle authority, with ROS operations injected by its caller.

Scene/trajectory messages are opaque to this module's imports. Operations must
plan against the supplied private scene and must never execute or apply it to
the live scene. A successful result proves the predicted cycle only; the caller
must still compare its live scene to initial_scene before reporting readiness.

The legacy top 2F path is preserved. Side-grip candidates use the same physical
cycle with their catalog-defined lateral contact corridor. Pinch candidates
use object-aligned geometry and finger-closing aperture. Arbitrary v2
grasp/place constraints remain unsupported.
"""
import copy
import math
import time
from dataclasses import dataclass
from typing import Callable

from perceived_object_grasp_plan import (
    build_grasp_target,
    oriented_box_extents,
    oriented_box_surface_distance,
    rotate_vector,
    tool_pose_for_grasp,
    compose_pose,
    inverse_pose,
    quaternion_from_rpy,
)
from physical_destination import check_object_containment
from grasp_strategy_candidates import generate_strategy_candidates


@dataclass(frozen=True)
class PreplanOperations:
    """Plan-only capabilities; updated_state includes installed gripper mimics.

    plan_segment returns the legacy motion record (before/after private scenes,
    trajectory, metadata), or raises when collision planning/corridor validation
    fails. state_validity returns measured contacts for the predicted RobotState
    against the unchanged live obstacles. Scene helpers return private copies.
    """
    plan_segment: Callable
    fk: Callable
    state_validity: Callable
    updated_state: Callable
    pose_message: Callable
    translated_pose: Callable
    target_contact_matrix: Callable
    verify_selected_contacts: Callable
    private_attachment: Callable
    object_pose_after_motion: Callable
    place_detachment_diff: Callable
    stage: Callable


@dataclass
class PreplanResult:
    success: bool
    candidate_id: str | None
    reason_code: str | None
    reason: str | None
    checks: list[dict]
    stages: list[dict]
    cycle: dict | None


class SearchBudgetExhausted(RuntimeError):
    """The global candidate-search wall-clock budget ended."""


class CandidateBudgetExhausted(RuntimeError):
    """One candidate used its fair-share discovery/retry wall-clock slice."""


class MotionFeasibilityFailure(RuntimeError):
    """Serializable rejection evidence from the existing collision/planning API."""

    def __init__(self, reason, *, moveit_code=None, contacts=()):
        super().__init__(reason)
        pairs = [dict(body_1=c.contact_body_1, body_2=c.contact_body_2,
                      body_type_1=c.body_type_1, body_type_2=c.body_type_2,
                      depth_m=c.depth) for c in contacts]
        self.details = dict(
            failure_kind='collision' if pairs else 'planning', moveit_code=moveit_code,
            contacts=pairs,
            colliding_links=sorted({p[f'body_{i}'] for p in pairs for i in (1, 2)
                                    if p[f'body_type_{i}'] == 0}),
            collision_objects=sorted({p[f'body_{i}'] for p in pairs for i in (1, 2)
                                      if p[f'body_type_{i}'] in (1, 2)}))


def preplan_full_cycle(*, initial_scene, observation: dict, candidate,
                       destination: dict, contract: dict, operations: PreplanOperations,
                       deadline: float) -> PreplanResult:
    """Predict all eleven steps or return the first failure without a cycle.

    stages contains the nine motion diagnostics used by the legacy runtime.
    cycle.steps also carries attach/detach records and opaque execution payloads.
    checks is additive evidence, never a substitute for the caller's live-scene
    comparison. The deadline is a time.monotonic() absolute search deadline.
    """
    steps, stages, checks = [], [], []
    view = copy.deepcopy(initial_scene)
    baseline = copy.deepcopy(initial_scene.allowed_collision_matrix)
    current_stage = None
    check_code = 'GENERATE_GRASPS'

    def stage(name):
        nonlocal current_stage, check_code
        check_code = name
        if current_stage != name:
            current_stage = name
            operations.stage(name)

    def budget_failure():
        if (contract.get('search_deadline') is not None and
                time.monotonic() >= contract['search_deadline']):
            return SearchBudgetExhausted('candidate search budget exhausted')
        return CandidateBudgetExhausted('candidate wall-clock slice exhausted')

    def motion(name, goal, group=None, straight=False):
        nonlocal view
        stage(name)
        if time.monotonic() > deadline:
            raise budget_failure()
        options = {}
        if name == 'PREPLAN_APPROACH' and contract.get('approach_ik') is not None:
            options['ik_binding'] = contract['approach_ik']
        if name == 'PREPLAN_TRANSFER' and contract.get('transfer_ik_seed') is not None:
            options['ik_seed'] = contract['transfer_ik_seed']
        step = operations.plan_segment(view, name, goal, group, straight, **options)
        if (step['metadata'].get('success') is not True or
                step['metadata'].get('moveit_code') != 1 or
                not step['metadata'].get('points') or step.get('trajectory') is None):
            raise RuntimeError('motion lacks successful collision-planning evidence')
        steps.append(step)
        stages.append(step['metadata'])
        view = copy.deepcopy(step['after'])
        checks.append(dict(code=name, status='PASS'))

    try:
        stage('GENERATE_GRASPS')
        intent = contract.get('task_intent')
        if intent is not None:
            grasp = intent['pick']['grasp']
            placement = intent['place']['placement']
            expected = {'top_2f': ('z_down', 'vertical'),
                        'side_grip_basic': ('x_plus', 'horizontal'),
                        'finger_pinch_basic': ('tool_z', 'tool_aligned')}[candidate.strategy_ref]
            # Do not silently discard constraints the current physical planner cannot honor.
            if (grasp['approach']['axis'] != expected[0] or
                    grasp['orientation']['mode'] != expected[1] or
                    grasp['lift']['axis'] != 'z_up' or
                    placement['approach']['axis'] != 'z_down' or
                    placement['retreat']['axis'] != 'z_up' or
                    placement['orientation']['mode'] != 'target_default'):
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: authored axes/orientation differ from supported cycle')
            if any(abs(v) > 1e-12 for key in ('tcp_offset_xyz_m', 'tcp_offset_rpy_rad')
                   for v in grasp.get(key, [0., 0., 0.])):
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: nonzero authored TCP adjustment')
            if grasp.get('contact', {}).get('min_quality', 0.) != 0.:
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: contact quality measurement unavailable')
            if grasp.get('orientation', {}).get('allowed_roll_deg', [0]) != [0]:
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: authored roll constraint')
            if any(abs(v) > 1e-12 for block in (grasp['orientation'], placement['orientation'])
                   for v in block.get('tolerance_rad', [0., 0., 0.])):
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: custom orientation tolerances')
            if candidate.strategy_ref in ('top_2f', 'finger_pinch_basic', 'side_grip_basic'):
                index = int(candidate.candidate_id.rsplit('::', 1)[1])
                yaw = ((index // 2) * 90 + (180 if index % 2 else 0)) % 360 if candidate.strategy_ref == 'top_2f' else index * 90
                if yaw not in grasp['orientation'].get('allowed_yaw_deg', [0, 90, 180, 270]):
                    raise RuntimeError('TASK_CONSTRAINT_UNSATISFIED: candidate yaw excluded by authored intent')
            if any(abs(v) > 1e-12 for v in placement['orientation'].get('rpy_rad', [0., 0., 0.])):
                raise RuntimeError('TASK_CONSTRAINT_UNSUPPORTED: authored placement orientation offset')
        if time.monotonic() > deadline:
            raise budget_failure()
        freshness_reference = contract.get('observation_reference_time', time.time())
        if (not isinstance(freshness_reference, (int, float)) or
                not math.isfinite(freshness_reference)):
            raise RuntimeError('invalid observation freshness reference')
        if (observation['timestamp'] > freshness_reference + .05 or
                freshness_reference - observation['timestamp'] > contract['max_age_seconds']):
            raise RuntimeError('observation expired before candidate planning')
        if candidate.object_id != observation['id']:
            raise RuntimeError('candidate object differs from observation')
        geometry = build_grasp_target(observation)
        extents = oriented_box_extents(geometry)
        if candidate.strategy_ref == 'top_2f':
            if set(candidate.effective) - {'approach_distance_m'}:
                raise RuntimeError('unsupported candidate constraints in legacy full-cycle extraction')
        elif candidate.strategy_ref == 'side_grip_basic':
            required_effective = {
                'approach_axis': 'x_plus',
                'orientation_mode': 'horizontal',
            }
            if (set(candidate.effective) != set(required_effective) | {'approach_distance_m'} or
                    any(candidate.effective.get(key) != value
                        for key, value in required_effective.items())):
                raise RuntimeError('unsupported or incompatible side-grip candidate constraints')
            distance = candidate.effective['approach_distance_m']
            displacement = [a-b for a, b in zip(candidate.approach_pose[:3], candidate.grasp_pose[:3])]
            tool_z = rotate_vector(candidate.grasp_pose[3:], [0.0, 0.0, 1.0])
            expected_contact = list(observation['pose'][:3])
            expected_contact[0] += oriented_box_surface_distance(geometry, [1.0, 0.0, 0.0])
            if (not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance < 0 or
                    math.dist(displacement, [distance, 0.0, 0.0]) > 1e-9 or
                    math.dist(candidate.grasp_pose[:3], expected_contact) > 1e-9 or
                    math.dist(tool_z, [-1.0, 0.0, 0.0]) > 1e-9):
                raise RuntimeError('side-grip candidate geometry is not x_plus/horizontal')
        elif candidate.strategy_ref == 'finger_pinch_basic':
            # Reuse the canonical geometry authority to reject relabelled or
            # unconsumed candidate constraints before any physical operation.
            expected = generate_strategy_candidates(
                candidate.strategy_ref, observation, candidate.effective)
            if not any(math.dist(candidate.grasp_pose, item.grasp_pose) < 1e-9 and
                       math.dist(candidate.approach_pose, item.approach_pose) < 1e-9
                       for item in expected):
                raise RuntimeError('pinch candidate geometry is not tool_z/tool_aligned')
        else:
            raise RuntimeError('unsupported grasp strategy in full-cycle preplanner')
        closing_world = rotate_vector(candidate.grasp_pose[3:], [0.0, 1.0, 0.0])
        qx, qy, qz, qw = observation['pose'][3:]
        closing_local = rotate_vector([-qx, -qy, -qz, qw], closing_world)
        aperture_extent = sum(abs(axis) * extent for axis, extent in
                              zip(closing_local, observation['dimensions']))
        if not math.isfinite(contract['retreat_distance_m']) or contract['retreat_distance_m'] <= 0:
            raise RuntimeError('retreat distance must be finite and positive')
        if intent is not None:
            aperture = intent['pick']['grasp'].get('aperture', {'min_m': 0., 'max_m': 0.085})
            if not aperture['min_m'] <= aperture_extent <= aperture['max_m']:
                raise RuntimeError('TASK_CONSTRAINT_UNSATISFIED: object outside authored aperture')
        if aperture_extent > 0.085:
            raise RuntimeError('target exceeds Robotiq aperture')
        if any(a > b for a, b in zip(extents, destination['dimensions'])):
            raise RuntimeError('target exceeds destination bounds')
        original = next(o for o in view.world.collision_objects if o.id == observation['id'])
        retreat = contract['retreat_distance_m']
        place_approach = contract.get('place_approach_distance_m', retreat)
        place_retreat = contract.get('place_retreat_distance_m', retreat)
        clearance = contract.get('placement_clearance_m', 0.001)
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0
               for v in (place_approach, place_retreat, clearance)):
            raise RuntimeError('invalid authored placement distances')
        home = dict(zip(contract['home_joint_names'], contract['home_joint_positions']))
        approach = operations.pose_message(tool_pose_for_grasp(candidate.approach_pose, contract))
        contact = operations.pose_message(tool_pose_for_grasp(candidate.grasp_pose, contract))
        motion('PREPLAN_APPROACH', approach)
        view.allowed_collision_matrix = operations.target_contact_matrix(
            baseline, observation['id'], contract['allowed_touch_links'])
        motion('PREPLAN_GRASP', contact, straight=True)
        stage('PREPLAN_CLOSE_GRIPPER')
        close = None
        reserve = contract.get('simulator_closure_reserve') is True
        required_contacts = set(contract['allowed_touch_links'])
        first_opposing = None
        if reserve and len(required_contacts) < 2:
            raise RuntimeError('simulator closure requires opposing fingertip links')
        for i in range(1, 81):
            trial = operations.updated_state(view.robot_state, {'gripper_finger1_joint': 0.804*i/80})
            response = operations.state_validity(trial)
            if response.contacts:
                try:
                    operations.verify_selected_contacts(response.contacts, observation['id'], contract['allowed_touch_links'])
                except RuntimeError as exc:
                    raise MotionFeasibilityFailure(str(exc), contacts=response.contacts) from exc
                if not reserve:
                    close = 0.804*i/80
                    break
            if reserve:
                planned_contacts = {
                    c.contact_body_2 if c.contact_body_1 == observation['id'] else c.contact_body_1
                    for c in response.contacts
                    if observation['id'] in (c.contact_body_1, c.contact_body_2)
                }
                opposing = required_contacts.issubset(planned_contacts)
                if first_opposing is not None:
                    if not opposing:
                        raise RuntimeError('opposing fingertip contact lost at closure reserve endpoint')
                    close = 0.804*i/80
                    break
                if opposing:
                    if i == 80:
                        raise RuntimeError('no closure reserve remains within gripper range')
                    first_opposing = 0.804*i/80
        if close is None:
            raise RuntimeError('no allowed fingertip contact in closing range')
        motion('PREPLAN_CLOSE_GRIPPER', {'gripper_finger1_joint': close}, group='gripper')
        if reserve:
            # A position reserve is not a force measurement. Execution must
            # still establish and continuously retain actual opposing contact.
            steps[-1]['metadata'].update(
                first_opposing_position_rad=first_opposing,
                commanded_position_rad=close,
                position_reserve_rad=close-first_opposing,
                required_contact_links=sorted(required_contacts),
                planned_contact_links=sorted(planned_contacts))
        tool_at_grasp = operations.fk(view.robot_state, contract['tool_link'])
        frame_at_grasp = operations.fk(view.robot_state, contract['grasp_frame'])
        stage('ATTACH')
        before = copy.deepcopy(view)
        view = operations.private_attachment(view, original, contract['grasp_frame'],
                                             frame_at_grasp.pose, contract['allowed_touch_links'])
        view.allowed_collision_matrix = copy.deepcopy(baseline)
        steps.append(dict(kind='attach', stage='ATTACH', before=before, after=copy.deepcopy(view), original=original))
        checks.append(dict(code='ATTACH', status='PASS'))
        # A reachable lift endpoint does not prove the required retreat corridor.
        # Use the same swept tool/robot check as descent before admitting a grasp.
        motion('PREPLAN_LIFT', operations.translated_pose(tool_at_grasp, dz=retreat), straight=True)
        delta = [a-b for a, b in zip(destination['pose_xyz'], observation['pose'][:3])]
        place_goal = operations.translated_pose(tool_at_grasp, *delta)
        if intent is not None:
            # Preserve the actual grasp transform while meeting the authored
            # destination orientation. Translation alone cannot reorient a part.
            p, q = tool_at_grasp.pose.position, tool_at_grasp.pose.orientation
            grasp_tool = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            object_in_tool = compose_pose(inverse_pose(grasp_tool), observation['pose'])
            destination_object = destination['pose_xyz'] + quaternion_from_rpy(destination['pose_rpy'])
            place_goal = operations.pose_message(compose_pose(destination_object, inverse_pose(object_in_tool)))
        motion('PREPLAN_TRANSFER', operations.translated_pose(place_goal, dz=place_approach))
        motion('PREPLAN_PLACE', place_goal)
        reached = operations.fk(view.robot_state, contract['tool_link'])
        achieved = operations.object_pose_after_motion(original, tool_at_grasp.pose, reached.pose)
        if math.dist(achieved[:3], destination['pose_xyz']) > 0.003:
            raise RuntimeError('planned placement differs from destination by more than 3 mm')
        check_code = 'DESTINATION_CONTAINMENT'
        if intent is not None:
            expected_orientation = quaternion_from_rpy(destination['pose_rpy'])
            angle = 2 * math.acos(min(1., abs(sum(a*b for a, b in zip(achieved[3:], expected_orientation)))))
            if angle > 0.01:
                raise RuntimeError('TASK_CONSTRAINT_UNSATISFIED: planned placement orientation differs from physical destination')
        check_object_containment(destination, achieved, list(original.primitives[0].dimensions), clearance=clearance)
        checks.append(dict(code=check_code, status='PASS'))
        motion('PREPLAN_OPEN_GRIPPER', {'gripper_finger1_joint': 0.0}, group='gripper')
        stage('DETACH')
        before = copy.deepcopy(view)
        placed = operations.place_detachment_diff(original, contract['grasp_frame'], achieved[:3], achieved[3:]).world.collision_objects[0]
        view = copy.deepcopy(view)
        view.robot_state.attached_collision_objects = []
        view.world.collision_objects.append(placed)
        steps.append(dict(kind='detach', stage='DETACH', before=before, after=copy.deepcopy(view),
                          original=original, tool_at_grasp=tool_at_grasp))
        checks.append(dict(code='DETACH', status='PASS'))
        view.allowed_collision_matrix = operations.target_contact_matrix(baseline, observation['id'], contract['allowed_touch_links'])
        motion('PREPLAN_RETREAT', operations.translated_pose(reached, dz=place_retreat), straight=True)
        view.allowed_collision_matrix = copy.deepcopy(baseline)
        motion('PREPLAN_HOME', home)
        stage('CANDIDATE_READY')
        cycle = dict(object_id=observation['id'], candidate=copy.deepcopy(candidate), steps=steps,
                     full_cycle_prevalidated=True)
        return PreplanResult(True, candidate.candidate_id, None, None, checks, stages, cycle)
    except Exception as exc:
        if isinstance(exc, SearchBudgetExhausted):
            reason_code = 'SEARCH_BUDGET_EXHAUSTED'
            failure = dict(failure_kind='budget')
        elif isinstance(exc, CandidateBudgetExhausted):
            reason_code = 'CANDIDATE_SLICE_EXHAUSTED'
            failure = dict(failure_kind='budget')
        else:
            reason_code = check_code + '_FAILED'
            failure = dict(failure_kind='planning' if check_code.startswith('PREPLAN_') else 'constraint')
        failure.update(getattr(exc, 'details', {}))
        failure.update(candidate_id=candidate.candidate_id, failed_stage=current_stage)
        stages.append(dict(stage=current_stage, success=False, reason=str(exc), reason_code=reason_code, **failure))
        checks.append(dict(code=check_code, status='FAIL', reason_code=reason_code, reason=str(exc), **failure))
        return PreplanResult(False, candidate.candidate_id, reason_code, str(exc), checks, stages, None)
