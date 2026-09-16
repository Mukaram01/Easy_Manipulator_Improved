#!/usr/bin/env python3
"""One plan-only full-cycle authority, with ROS operations injected by its caller.

Scene/trajectory messages are opaque to this module's imports. Operations must
plan against the supplied private scene and must never execute or apply it to
the live scene. A successful result proves the predicted cycle only; the caller
must still compare its live scene to initial_scene before reporting readiness.

The initial extraction preserves the legacy top 2F contact scan and vertical
corridor. It does not implement arbitrary v2 grasp/place constraints.
"""
import copy
import math
import time
from dataclasses import dataclass
from typing import Callable

from perceived_object_grasp_plan import build_grasp_target, oriented_box_extents, tool_pose_for_grasp
from physical_destination import check_object_containment


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

    def motion(name, goal, group=None, straight=False):
        nonlocal view
        stage(name)
        if time.monotonic() > deadline:
            raise RuntimeError('candidate search budget exhausted')
        step = operations.plan_segment(view, name, goal, group, straight)
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
        if time.monotonic() > deadline:
            raise RuntimeError('candidate search budget exhausted')
        if time.time() - observation['timestamp'] > contract['max_age_seconds']:
            raise RuntimeError('observation expired before candidate planning')
        if candidate.object_id != observation['id']:
            raise RuntimeError('candidate object differs from observation')
        if candidate.strategy_ref != 'top_2f':
            raise RuntimeError('unsupported grasp strategy in legacy full-cycle extraction')
        if not math.isfinite(contract['retreat_distance_m']) or contract['retreat_distance_m'] <= 0:
            raise RuntimeError('retreat distance must be finite and positive')
        extents = oriented_box_extents(build_grasp_target(observation))
        if min(extents[:2]) > 0.085:
            raise RuntimeError('target exceeds Robotiq aperture')
        if any(a > b for a, b in zip(extents, destination['dimensions'])):
            raise RuntimeError('target exceeds destination bounds')
        original = next(o for o in view.world.collision_objects if o.id == observation['id'])
        retreat = contract['retreat_distance_m']
        home = dict(zip(contract['home_joint_names'], contract['home_joint_positions']))
        approach = operations.pose_message(tool_pose_for_grasp(candidate.approach_pose, contract))
        contact = operations.pose_message(tool_pose_for_grasp(candidate.grasp_pose, contract))
        motion('PREPLAN_APPROACH', approach)
        view.allowed_collision_matrix = operations.target_contact_matrix(
            baseline, observation['id'], contract['allowed_touch_links'])
        motion('PREPLAN_GRASP', contact, straight=True)
        stage('PREPLAN_CLOSE_GRIPPER')
        close = None
        for i in range(1, 81):
            trial = operations.updated_state(view.robot_state, {'gripper_finger1_joint': 0.804*i/80})
            response = operations.state_validity(trial)
            if response.contacts:
                operations.verify_selected_contacts(response.contacts, observation['id'], contract['allowed_touch_links'])
                close = 0.804*i/80
                break
        if close is None:
            raise RuntimeError('no allowed fingertip contact in closing range')
        motion('PREPLAN_CLOSE_GRIPPER', {'gripper_finger1_joint': close}, group='gripper')
        tool_at_grasp = operations.fk(view.robot_state, contract['tool_link'])
        frame_at_grasp = operations.fk(view.robot_state, contract['grasp_frame'])
        stage('ATTACH')
        before = copy.deepcopy(view)
        view = operations.private_attachment(view, original, contract['grasp_frame'],
                                             frame_at_grasp.pose, contract['allowed_touch_links'])
        view.allowed_collision_matrix = copy.deepcopy(baseline)
        steps.append(dict(kind='attach', stage='ATTACH', before=before, after=copy.deepcopy(view), original=original))
        checks.append(dict(code='ATTACH', status='PASS'))
        motion('PREPLAN_LIFT', operations.translated_pose(tool_at_grasp, dz=retreat))
        delta = [a-b for a, b in zip(destination['pose_xyz'], observation['pose'][:3])]
        motion('PREPLAN_TRANSFER', operations.translated_pose(tool_at_grasp, delta[0], delta[1], delta[2]+retreat))
        motion('PREPLAN_PLACE', operations.translated_pose(tool_at_grasp, *delta))
        reached = operations.fk(view.robot_state, contract['tool_link'])
        achieved = operations.object_pose_after_motion(original, tool_at_grasp.pose, reached.pose)
        if math.dist(achieved[:3], destination['pose_xyz']) > 0.003:
            raise RuntimeError('planned placement differs from destination by more than 3 mm')
        check_code = 'DESTINATION_CONTAINMENT'
        check_object_containment(destination, achieved, list(original.primitives[0].dimensions), clearance=0.001)
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
        motion('PREPLAN_RETREAT', operations.translated_pose(reached, dz=retreat), straight=True)
        view.allowed_collision_matrix = copy.deepcopy(baseline)
        motion('PREPLAN_HOME', home)
        stage('CANDIDATE_READY')
        cycle = dict(object_id=observation['id'], candidate=copy.deepcopy(candidate), steps=steps,
                     full_cycle_prevalidated=True)
        return PreplanResult(True, candidate.candidate_id, None, None, checks, stages, cycle)
    except Exception as exc:
        reason_code = check_code + '_FAILED'
        stages.append(dict(stage=current_stage, success=False, reason=str(exc), reason_code=reason_code))
        checks.append(dict(code=check_code, status='FAIL', reason_code=reason_code, reason=str(exc)))
        return PreplanResult(False, candidate.candidate_id, reason_code, str(exc), checks, stages, None)
