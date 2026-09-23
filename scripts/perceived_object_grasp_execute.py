#!/usr/bin/env python3
"""Plan, execute, attach, and retreat for one perceived object on fake hardware."""

import argparse
import copy
import importlib.util
import json
import math
import signal
import time
from contextlib import contextmanager
from pathlib import Path


_PLANNER_PATH = Path(__file__).with_name("perceived_object_grasp_plan.py")
_SPEC = importlib.util.spec_from_file_location("p8e1_grasp_plan", _PLANNER_PATH)
_PLANNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PLANNER)

ARM_JOINT_SUFFIXES = _PLANNER.ARM_JOINT_SUFFIXES
select_perceived_box = _PLANNER.select_perceived_box
build_grasp_target = _PLANNER.build_grasp_target
generate_box_grasp_candidates = _PLANNER.generate_box_grasp_candidates
oriented_box_extents = _PLANNER.oriented_box_extents
collision_object_dict = _PLANNER._collision_object_dict


def select_graspable_box(objects, max_aperture=0.085, max_planar_extent=None):
    """Select deterministically among boxes that physically fit the 2F gripper."""
    feasible = []
    for obj in objects:
        try:
            extents = oriented_box_extents(build_grasp_target(obj))
        except (KeyError, TypeError, ValueError):
            continue
        # The aperture constrains the horizontal cross-section after the live
        # object orientation is applied; its vertical height is irrelevant to
        # the finger opening.
        if (min(extents[:2]) <= max_aperture
                and (max_planar_extent is None or max(extents[:2]) <= max_planar_extent)):
            feasible.append(obj)
    if not feasible:
        raise ValueError("no perceived BOX fits the Robotiq 2F grasp envelope")
    return select_perceived_box(feasible)


def support_penetration_correction(selected, objects, clearance=0.001):
    """Return a diagnostic support-contact correction for legacy callers.

    The execution path deliberately does not rewrite a live perception pose.
    Keeping this pure helper preserves the older offline contract while
    preventing authored or fabricated geometry from entering the PlanningScene.
    """
    sx, sy, sz = selected["pose"][:3]
    sdx, sdy, sdz = selected["dimensions"]
    selected_bottom = sz - 0.5 * sdz
    best = None
    for other in objects:
        if other["id"] == selected["id"]:
            continue
        ox, oy, oz = other["pose"][:3]
        odx, ody, odz = other["dimensions"]
        other_top = oz + 0.5 * odz
        xy_supported = (abs(sx - ox) <= 0.5 * (sdx + odx)
                        and abs(sy - oy) <= 0.5 * (sdy + ody))
        penetration = other_top - selected_bottom
        if xy_supported and 0.0 < penetration <= 0.5 * sdz:
            candidate = (penetration + clearance, other["id"])
            if best is None or candidate[0] < best[0]:
                best = candidate
    return best


def load_canonical_place_target(package_share, destination_zone=None):
    import yaml
    package = Path(package_share)
    cell_path = package / "cell_definition.yaml"
    from physical_destination import resolve_destination
    if not cell_path.exists():
        raise RuntimeError(f"generated cell handoff is missing: {cell_path}")
    cell = yaml.safe_load(cell_path.read_text(encoding="utf-8")) or {}
    task = cell.get("task") or {}
    zone_id = destination_zone or str((task.get("place") or {}).get("target_ref") or "")
    result = resolve_destination(cell.get("environment") or {}, zone_id)
    authored_path = package / "environment.yaml"
    if authored_path.exists():
        authored = yaml.safe_load(authored_path.read_text(encoding="utf-8")) or {}
        expected = resolve_destination(authored.get("environment") or {}, zone_id)
        if result != expected:
            raise RuntimeError("stale generated physical destination differs from authored environment")
    destination = next((d for d in task.get("destinations", []) if d.get("id") == zone_id), {})
    if destination.get("target_ref", result['target_id']) != result['target_id']:
        raise RuntimeError("task destination target_ref differs from physical zone owner")
    return result


def fake_hardware_evidence(parameter_values, hardware_components):
    """Fail closed unless MoveIt and controller_manager both prove mock hardware."""
    fake_parameter = bool(parameter_values and parameter_values[0].bool_value)
    classes = [component.class_type for component in hardware_components]
    mock_classes = [name for name in classes if name == "mock_components/GenericSystem"]
    real_markers = ("URPositionHardwareInterface", "ur_robot_driver", "EtherCAT", "RealSystem")
    real_classes = [name for name in classes if any(marker.lower() in name.lower()
                                                      for marker in real_markers)]
    if not fake_parameter or not mock_classes or len(mock_classes) != len(classes) or real_classes:
        raise RuntimeError(
            "execution rejected: fake hardware was not exclusively proven active "
            f"(move_group={fake_parameter}, classes={classes})"
        )
    return {"move_group_use_fake_hardware": True,
            "hardware_classes": classes, "real_hardware": False}


def target_contact_matrix(baseline, object_id, touch_links):
    """Preserve the full ACM and permit only selected-object/contact-link pairs."""
    from moveit_msgs.msg import AllowedCollisionEntry
    if not object_id or not touch_links or object_id in touch_links:
        raise ValueError("contact transition requires one target and explicit contact links")
    matrix = copy.deepcopy(baseline)
    names = list(matrix.entry_names)
    if len(set(names)) != len(names) or len(matrix.entry_values) != len(names) or any(
            len(row.enabled) != len(names) for row in matrix.entry_values):
        raise ValueError("PlanningScene ACM is not a unique square matrix")
    if any(link not in names for link in touch_links):
        raise ValueError("contact link is absent from the PlanningScene ACM")
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    if defaults.get(object_id, False):
        raise ValueError("selected object has a broad default collision allowance")
    if object_id not in names:
        # Preserve the effective defaults for existing names as the new row
        # becomes explicit. Reject pre-existing broad target exemptions below.
        values = [bool(defaults.get(name, False)) for name in names]
        for row, value in zip(matrix.entry_values, values):
            row.enabled.append(value)
        matrix.entry_values.append(AllowedCollisionEntry(enabled=values + [False]))
        matrix.entry_names.append(object_id)
        names.append(object_id)
    target = names.index(object_id)
    for index, name in enumerate(names):
        if name not in touch_links and name != object_id and (
                matrix.entry_values[target].enabled[index]
                or matrix.entry_values[index].enabled[target]):
            raise ValueError(f"selected object already permits non-contact collision: {name}")
    for link in touch_links:
        index = names.index(link)
        matrix.entry_values[target].enabled[index] = True
        matrix.entry_values[index].enabled[target] = True
    return matrix


def collision_matrix_signature(matrix):
    """MoveIt may reorder ACM entries when serializing its internal map."""
    return (sorted(matrix.entry_names),
            sorted((a, b, bool(matrix.entry_values[i].enabled[j]))
                   for i, a in enumerate(matrix.entry_names)
                   for j, b in enumerate(matrix.entry_names)),
            sorted(zip(matrix.default_entry_names, matrix.default_entry_values)))


@contextmanager
def temporary_target_contact(baseline, object_id, touch_links, apply_matrix):
    """Restore the original ACM on success, planning failure, or execution failure."""
    contact = target_contact_matrix(baseline, object_id, touch_links)
    try:
        apply_matrix(contact)
        yield
    finally:
        apply_matrix(copy.deepcopy(baseline))


def attachment_diff(original, link_name, touch_links, remove_world=True):
    """Build one atomic world-remove/robot-attach PlanningScene diff."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

    scene = PlanningScene()
    scene.is_diff = True
    if remove_world:
        remove = CollisionObject()
        remove.id = original.id
        remove.header = copy.deepcopy(original.header)
        remove.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(remove)

    attached = AttachedCollisionObject()
    attached.link_name = link_name
    attached.touch_links = list(touch_links)
    attached.object = copy.deepcopy(original)
    attached.object.operation = CollisionObject.ADD
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects.append(attached)
    return scene


def attachment_status(scene, object_id, link_name):
    world_ids = [obj.id for obj in scene.world.collision_objects]
    attached = [item for item in scene.robot_state.attached_collision_objects
                if item.object.id == object_id]
    return {
        "world_object_present": object_id in world_ids,
        "attached_object_present": len(attached) == 1,
        "attached_link": attached[0].link_name if len(attached) == 1 else "",
        "valid": object_id not in world_ids and len(attached) == 1
                 and attached[0].link_name == link_name,
    }


def detachment_cleanup_diff(object_id, link_name):
    """Remove an attachment after any post-attach failure."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

    cleanup = PlanningScene()
    cleanup.is_diff = True
    cleanup.robot_state.is_diff = True
    item = AttachedCollisionObject()
    item.link_name = link_name
    item.object.id = object_id
    item.object.operation = CollisionObject.REMOVE
    cleanup.robot_state.attached_collision_objects.append(item)
    return cleanup


def place_detachment_diff(original, link_name, place_xyz, place_orientation=None):
    """Detach the selected ID and restore it once at the decided world target."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    detached = AttachedCollisionObject()
    detached.link_name = link_name
    detached.object.id = original.id
    detached.object.operation = CollisionObject.REMOVE
    scene.robot_state.attached_collision_objects.append(detached)
    placed = copy.deepcopy(original)
    placed.operation = CollisionObject.ADD
    placed.pose.position.x, placed.pose.position.y, placed.pose.position.z = place_xyz
    if place_orientation is not None:
        (placed.pose.orientation.x, placed.pose.orientation.y,
         placed.pose.orientation.z, placed.pose.orientation.w) = place_orientation
    if (placed.pose.orientation.x ** 2 + placed.pose.orientation.y ** 2
            + placed.pose.orientation.z ** 2 + placed.pose.orientation.w ** 2) < 1e-12:
        placed.pose.orientation.w = 1.0
    scene.world.collision_objects.append(placed)
    return scene


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def pose_within_cartesian_corridor(actual, start, goal,
                                   position_tolerance=0.0025,
                                   orientation_tolerance=0.005):
    """Return whether a pose stays in the requested straight Cartesian tube."""
    direction = [b-a for a, b in zip(start[:3], goal[:3])]
    length_squared = sum(value * value for value in direction)
    if length_squared < 1e-18:
        position_ok = math.dist(actual[:3], goal[:3]) <= position_tolerance
    else:
        relative = [value-origin for value, origin in zip(actual[:3], start[:3])]
        fraction = sum(value * axis for value, axis in zip(relative, direction)) / length_squared
        length = math.sqrt(length_squared)
        if not -position_tolerance / length <= fraction <= 1.0 + position_tolerance / length:
            return False
        projection = [origin + fraction * axis for origin, axis in zip(start[:3], direction)]
        position_ok = math.dist(actual[:3], projection) <= position_tolerance
    orientation_error = min(
        math.dist(actual[3:], goal[3:]),
        math.dist(actual[3:], [-value for value in goal[3:]]),
    )
    return position_ok and orientation_error <= orientation_tolerance


def object_pose_after_motion(original, start_tool, end_tool):
    """Propagate the actual rigid attachment transform, including rotation."""
    base = pose_values(original.pose)
    if sum(v*v for v in base[3:]) < 1e-12:
        base[3:] = [0.0, 0.0, 0.0, 1.0]
    relative = _PLANNER.compose_pose(_PLANNER.inverse_pose(pose_values(start_tool)), base)
    return _PLANNER.compose_pose(pose_values(end_tool), relative)


def verify_selected_contacts(contacts, object_id, touch_links):
    """Require actual fingertip contact; reject palm, camera, or any other pair."""
    pairs = [{c.contact_body_1, c.contact_body_2} for c in contacts]
    allowed = [{object_id, link} for link in touch_links]
    if not pairs or any(pair not in allowed for pair in pairs):
        raise RuntimeError(f"grasp contact is absent or involves a non-contact link: {pairs}")


def translated_pose(pose, dx=0.0, dy=0.0, dz=0.0):
    """Return a copied tool pose translated in the planning frame."""
    result = copy.deepcopy(pose)
    result.pose.position.x += dx
    result.pose.position.y += dy
    result.pose.position.z += dz
    return result


def candidate_indices(count, preferred=3):
    """Try a preferred orientation, followed by every remaining candidate."""
    if not 0 <= preferred < count:
        return list(range(count))
    return [preferred] + [index for index in range(count) if index != preferred]


class MoveItActionFailure(RuntimeError):
    def __init__(self, status, code):
        super().__init__(f'MoveIt action failed: status={status}, code={code}')
        self.code = code


def retryable_plan_failure(code):
    """Retry only stochastic plan-only outcomes without changing scene or goal."""
    # MoveItErrorCodes: INVALID_MOTION_PLAN=-2, TIMED_OUT=-6.
    # A retry reuses the exact same MotionPlanRequest/PlanningScene/candidate.
    return code in (-2, -6)


def preplan_retryable_failure(result):
    """Return whether a failed preplan deserves a later identical planning retry.

    Discovery must not spend repeated stochastic planning windows on one candidate
    while later objects/strategies remain untested. Deterministic IK, collision,
    geometry, corridor and task-constraint failures are terminal for that candidate.
    """
    if result.success or result.reason_code == 'SEARCH_BUDGET_EXHAUSTED':
        return False
    if result.reason_code == 'CANDIDATE_SLICE_EXHAUSTED':
        return True
    # A later geometric rejection must not erase an earlier variant's
    # stochastic failure. The same existing candidate retry budget still caps
    # the whole evaluation; no variant receives an extra planning budget.
    if any(attempt.get('failure_kind') == 'planning' and
           retryable_plan_failure(attempt.get('moveit_code'))
           for attempt in getattr(result, 'extraction_attempts', [])):
        return True
    for check in reversed(result.checks):
        if check.get('status') not in ('FAIL', 'BLOCKED'):
            continue
        return (check.get('failure_kind') == 'planning' and
                retryable_plan_failure(check.get('moveit_code')))
    return False


class CandidateFailure(RuntimeError):
    def __init__(self, stage, reason):
        super().__init__(reason)
        self.stage = stage


def contacts_in_planned_scene(contacts, planned, live):
    """Keep diagnostic contacts whose geometry/permissions match the private plan.

    GetStateValidity reads the live world. Never mislabel a permitted pad contact
    or a removed/moved private object as the reason a private MoveGroup plan failed.
    This filters diagnostics only; MoveGroup's collision checks remain authoritative.
    """
    world = {o.id: o for o in planned.world.collision_objects}
    original = {o.id: o for o in live.world.collision_objects}
    matrix = planned.allowed_collision_matrix
    indices = {name: i for i, name in enumerate(matrix.entry_names)}
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    result = []
    for contact in contacts:
        a, b = contact.contact_body_1, contact.contact_body_2
        if any(kind == 1 and (name not in world or world[name] != original.get(name))
               for name, kind in ((a, contact.body_type_1), (b, contact.body_type_2))):
            continue
        pair_defaults = [defaults[name] for name in (a, b) if name in defaults]
        allowed = (matrix.entry_values[indices[a]].enabled[indices[b]]
                   if a in indices and b in indices else bool(pair_defaults) and all(pair_defaults))
        if not allowed:
            result.append(contact)
    return result


def legitimate_support_ids(observation, environment, manifest):
    """Bind only declared supports or the floor beneath reviewed usable space.

    This identifies eligible physical owners, not collision exemptions. Exact
    floor contacts/depth are checked by MoveIt's per-contact policy. In
    particular usable_placement's lower face is NOT the physical floor height.
    """
    result = set()
    declared = {a['id']: a for a in environment.get('support_surfaces', [])}
    assets = declared | {a['id']: a for a in environment.get('assets', [])}
    for item in manifest.get('objects', []):
        asset = assets.get(item.get('source_item_id'))
        if not asset or asset.get('frame') != 'world' or (asset.get('collision') or {}).get('enabled') is not True:
            continue
        # The supported lift is world +Z. Tilted support semantics require a
        # separately authored approach; do not guess them from a nearby contact.
        if any(abs(v) > 1e-9 for v in asset.get('pose_rpy', [0., 0., 0.])):
            continue
        usable = asset.get('usable_placement')
        if usable:
            if any(abs(v) > 1e-9 for v in usable.get('pose_rpy', [0., 0., 0.])):
                continue
            center = [a+b for a,b in zip(asset['pose_xyz'], usable['pose_xyz'])]
            extent = oriented_box_extents(build_grasp_target(observation))
            if (any(abs(p-c)+e/2 > d/2 for p,c,e,d in
                    zip(observation['pose'][:2], center[:2], extent[:2], usable['dimensions'][:2])) or
                    observation['pose'][2] > center[2]+usable['dimensions'][2]/2):
                continue
        elif asset['id'] not in declared:
            continue
        result.add(item['id'])
    return result


def ik_scene_matches_live(planned, live):
    """Whether the live IK service can check this private scene without drift."""
    return (planned.world == live.world
            and collision_matrix_signature(planned.allowed_collision_matrix)
            == collision_matrix_signature(live.allowed_collision_matrix)
            and planned.robot_state.attached_collision_objects
            == live.robot_state.attached_collision_objects)


def approach_ik_binding(goal, contract, positions):
    """Bind the selected IK branch to model, group, TCP and Cartesian target.

    This is a seed, never a trajectory or a replacement for current-state
    collision planning. It participates in the existing resolution digest.
    """
    return dict(schema='workcell_approach_ik/v1',
                robot_model_sha256=contract['robot_model_sha256'],
                planning_group=contract['planning_group'], tool_link=contract['tool_link'],
                frame_id=goal.header.frame_id, target_pose=pose_values(goal.pose),
                joint_positions={n: float(positions[n]) for n in contract['home_joint_names']})


def bound_approach_seed(binding, goal, contract, current, mimics):
    try:
        positions = binding['joint_positions']
        expected = approach_ik_binding(goal, contract, positions)
        if (binding != expected or set(positions) != set(contract['home_joint_names']) or
                not all(math.isfinite(v) for v in positions.values())):
            raise ValueError('binding differs')
        return updated_state(current, positions, mimics)
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError('APPROACH_IK_BINDING_CHANGED: resolve current model/target again') from exc


def transfer_ik_seed(goal, contract, positions):
    """A proven transfer arm branch for fresh IK/planning, never a trajectory."""
    arm = {n: positions[n] for n in contract['home_joint_names']}
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in arm.values()):
        raise ValueError('nonfinite or nonnumeric transfer arm seed')
    return dict(schema='workcell_transfer_ik_seed/v1',
                robot_model_sha256=contract['robot_model_sha256'],
                planning_group=contract['planning_group'], tool_link=contract['tool_link'],
                frame_id=goal.header.frame_id, stage='PREPLAN_TRANSFER',
                joint_positions={n: float(v) for n, v in arm.items()})


def bound_transfer_seed(seed, goal, contract, current, mimics):
    try:
        positions = seed['joint_positions']
        if (set(positions) != set(contract['home_joint_names']) or
                seed != transfer_ik_seed(goal, contract, positions)):
            raise ValueError('transfer seed context differs')
        return updated_state(current, positions, mimics)
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError('TRANSFER_IK_SEED_CHANGED: resolve current transfer context again') from exc


def choose_cycle(targets, indices, preplan, attempts):
    """First fully feasible pair in confidence/id then preferred-grasp order."""
    for target in sorted(targets, key=lambda o: (o['confidence'] is None, -(o['confidence'] or 0.0), o['id'])):
        for index in indices:
            record = {'object_id': target['id'], 'grasp_index': index, 'stages': []}
            attempts.append(record)
            try:
                result = preplan(target, index, record)
                record['full_cycle_prevalidated'] = True
                return result
            except CandidateFailure as exc:
                record.update(full_cycle_prevalidated=False, failed_stage=exc.stage, reason=str(exc))
    last = attempts[-1] if attempts else {}
    raise CandidateFailure(last.get('failed_stage','ENUMERATE_TARGETS'),
        'no target/grasp has a feasible complete cycle; last failure: ' + last.get('reason','no eligible targets'))


def plan_legacy_cycle(*, initial_scene, targets, destination, contract, operations, deadline, summary):
    """Adapt legacy confidence/index selection to the shared full-cycle authority.

    contract includes the task's max_age_seconds and effective retreat distance.
    Runtime owns live-scene verification and all execution gates after this call.
    """
    from grasp_strategy_candidates import generate_strategy_candidates
    from full_cycle_preplanner import preplan_full_cycle
    operations.stage('ENUMERATE_TARGETS')
    candidates = {target['id']: generate_strategy_candidates('top_2f', target,
                  {'approach_distance_m': contract['approach_distance_m']}) for target in targets}
    summary['grasp_candidates'] = {object_id: [list(c.grasp_pose) for c in choices]
                                   for object_id, choices in candidates.items()}
    summary['grasp_candidate_source'] = 'canonical_box_geometry_robotiq_2f'
    summary['grasp_candidate_count'] = sum(len(v) for v in candidates.values())

    def preplan(target, index, record):
        result = preplan_full_cycle(initial_scene=initial_scene, observation=target,
            candidate=candidates[target['id']][index], destination=destination,
            contract=contract, operations=operations, deadline=deadline)
        record['stages'].extend(result.stages)
        record['checks'] = result.checks
        record['candidate_id'] = result.candidate_id
        if not result.success:
            record['reason_code'] = result.reason_code
            raise CandidateFailure(result.stages[-1]['stage'], result.reason)
        result.cycle['grasp_index'] = index
        return result.cycle

    return choose_cycle(targets, candidate_indices(8), preplan, summary['candidate_attempts'])


def plan_authored_cycle(*, initial_scene, intent, environment, cell, targets,
                        contract, operations, deadline, summary, resolved=None,
                        planning_time=3.0):
    """Resolve with fair discovery, then spend retries only on stochastic failures.

    A fresh resolve gives every candidate one bounded planning window before any
    candidate receives an identical retry. Revalidation of an already-resolved
    handoff keeps the historical full planning budget because it checks exactly
    one bound candidate and is not a search.
    """
    from task_intent_resolver import resolve_task_intent, STRATEGY_ORDER
    from full_cycle_preplanner import preplan_full_cycle
    cycles = {}
    resolution_reference_time = time.time()
    evaluation_cache = {}
    discovery_order = {}
    discovery_time = min(float(planning_time), 0.75)
    # The resolver is intentionally phased by strategy. A PREFERRED strategy
    # gets a real bounded chance before fallback discovery, rather than paying
    # to sample every fallback candidate first. These bounds are chosen so the
    # complete current 10-part / 130-candidate Stage-A search can still sample
    # every strategy and retry a small progress-ranked beam inside 300 s.
    discovery_candidate_budget = 0.75
    retry_candidate_budget = 20.0
    retry_beam_width = 3

    def request_key(request):
        destination = request['destination']
        return (
            request['strategy_ref'],
            request['observation']['id'],
            request['candidate'].candidate_id,
            destination.get('id'),
            tuple(destination.get('pose_xyz', ())),
            tuple(destination.get('pose_rpy', ())),
        )

    def evaluate_once(request, *, search_pass, planning_attempts, segment_time,
                      candidate_budget=None):
        effective = dict(contract)
        started = time.monotonic()
        candidate_deadline = deadline
        if candidate_budget is not None:
            candidate_deadline = min(deadline, started + float(candidate_budget))
        effective['search_deadline'] = deadline
        if resolved is not None and request.get('approach_ik') is None:
            return dict(success=False, checks=[], reason_code='TASK_APPROACH_IK_UNBOUND',
                        reason='Resolve and Generate the current approach IK branch before consumption.',
                        retryable=False)
        if resolved is not None and request.get('transfer_ik_seed') is None:
            return dict(success=False, checks=[], reason_code='TASK_TRANSFER_IK_UNBOUND',
                        reason='Resolve and Generate the current transfer IK branch before consumption.',
                        retryable=False)
        effective['approach_ik'] = request.get('approach_ik')
        effective['transfer_ik_seed'] = request.get('transfer_ik_seed')
        effective['extraction_intent'] = request.get('extraction_intent')
        if resolved is not None and operations.extraction_candidates is not None and request.get('extraction_intent') is None:
            return dict(success=False, checks=[], reason_code='TASK_EXTRACTION_UNBOUND',
                        reason='Resolve and Generate the current extraction intent before consumption.', retryable=False)
        grasp = request['intent']['pick']['grasp']
        place = request['intent']['place']['placement']
        effective.update(observation_reference_time=resolution_reference_time,
                         max_age_seconds=intent['pick']['selection']['object_filter']['max_age_seconds'],
                         retreat_distance_m=grasp['lift']['distance_m'],
                         place_approach_distance_m=place['approach']['distance_m'],
                         place_retreat_distance_m=place['retreat']['distance_m'],
                         placement_clearance_m=place['clearance_m'],
                         task_intent=request['intent'])

        # plan_segment is the injected MoveIt boundary and closes over this
        # contract object. Temporarily bind search policy without changing the
        # serialized grasp contract or the preplanner's physical semantics.
        sentinel = object()
        old_attempts = contract.get('_candidate_planning_attempts', sentinel)
        old_time = contract.get('_candidate_planning_time', sentinel)
        old_pass = contract.get('_candidate_search_pass', sentinel)
        old_wall = contract.get('_candidate_wall_deadline', sentinel)
        contract['_candidate_planning_attempts'] = planning_attempts
        contract['_candidate_planning_time'] = segment_time
        contract['_candidate_search_pass'] = search_pass
        contract['_candidate_wall_deadline'] = candidate_deadline
        try:
            result = preplan_full_cycle(initial_scene=initial_scene,
                observation=request['observation'], candidate=request['candidate'],
                destination=request['destination'], contract=effective,
                operations=operations, deadline=candidate_deadline)
        finally:
            for key, old in (
                    ('_candidate_planning_attempts', old_attempts),
                    ('_candidate_planning_time', old_time),
                    ('_candidate_search_pass', old_pass),
                    ('_candidate_wall_deadline', old_wall)):
                if old is sentinel:
                    contract.pop(key, None)
                else:
                    contract[key] = old

        summary['candidate_attempts'].append({
            'candidate_id': result.candidate_id, 'object_id': request['observation']['id'],
            'stages': result.stages, 'checks': result.checks, 'reason_code': result.reason_code,
            'search_pass': search_pass, 'planning_attempts': planning_attempts,
            'segment_planning_time': segment_time,
            'candidate_wall_budget': candidate_budget,
            'candidate_wall_seconds': time.monotonic() - started,
            'extraction_attempts': copy.deepcopy(result.extraction_attempts)})
        if result.success:
            cycles[result.candidate_id, request['observation']['id']] = result.cycle
        progress_passes = sum(1 for check in result.checks if check.get('status') == 'PASS')
        failed_stage = next((check.get('failed_stage') or check.get('code')
                             for check in reversed(result.checks)
                             if check.get('status') in ('FAIL', 'BLOCKED')), None)
        approach_ik = None
        if result.success:
            approach_ik = result.cycle['steps'][0]['metadata'].get('approach_ik')
        else:
            # Discovery may prove a concrete approach IK branch and then use up
            # its fair wall-clock slice later in the cycle. Preserve that
            # already-proven branch for the retry instead of asking IK/OMPL to
            # sample a different stochastic approach from scratch.
            approach_stage = next((
                stage for stage in result.stages
                if stage.get('stage') == 'PREPLAN_APPROACH'
                and stage.get('success') is True
                and stage.get('approach_ik') is not None
            ), None)
            if approach_stage is not None:
                approach_ik = copy.deepcopy(approach_stage['approach_ik'])
        transfer_stage = next((stage for stage in result.stages
                               if stage.get('stage') == 'PREPLAN_TRANSFER'
                               and stage.get('success') is True
                               and stage.get('transfer_ik_seed') is not None), None)
        return {'success': result.success, 'checks': result.checks,
                'reason_code': result.reason_code, 'reason': result.reason,
                'approach_ik': approach_ik,
                'extraction_intent': copy.deepcopy(result.cycle.get('extraction_intent')) if result.success else None,
                'extraction_attempts': copy.deepcopy(result.extraction_attempts),
                # Only complete candidates bind transfer IK; failed variant
                # seeds must never escape into another extraction alternative.
                'transfer_ik_seed': copy.deepcopy(transfer_stage['transfer_ik_seed']) if result.success and transfer_stage else None,
                'retryable': preplan_retryable_failure(result),
                'stop_search': result.reason_code == 'SEARCH_BUDGET_EXHAUSTED',
                'progress_passes': progress_passes,
                'failed_stage': failed_stage}

    if resolved is not None:
        resolution = resolve_task_intent(
            intent, environment, cell, targets,
            lambda request: evaluate_once(request, search_pass='revalidate',
                                          planning_attempts=3, segment_time=float(planning_time),
                                          candidate_budget=None),
            now=resolution_reference_time, resolved=resolved)
        summary['candidate_search'] = {
            'mode': 'resolved_revalidation',
            'retry_pass_used': False,
            'full_planning_time': float(planning_time),
        }
    else:
        grasp_policy = intent['pick']['grasp']['policy']
        requested_strategy = intent['pick']['grasp'].get('strategy_ref')
        if grasp_policy == 'EXACT':
            strategy_phases = [requested_strategy]
        elif grasp_policy == 'PREFERRED':
            strategy_phases = [requested_strategy] + [
                strategy for strategy in STRATEGY_ORDER if strategy != requested_strategy]
        else:
            strategy_phases = list(STRATEGY_ORDER)

        retry_attempted = []
        phase_evidence = []
        resolution = None
        stop_all = False

        def phase_boundary(strategy):
            return {
                'success': False,
                'checks': [],
                'reason_code': 'STRATEGY_PHASE_COMPLETE',
                'reason': f'Bounded discovery for {strategy} is complete; retry before fallback.',
                'retryable': False,
                'stop_search': True,
                'progress_passes': 0,
                'failed_stage': None,
            }

        for phase_index, strategy in enumerate(strategy_phases):
            phase_retryable = set()
            phase_discovered = []

            def discover(request, strategy=strategy):
                key = request_key(request)
                cached = evaluation_cache.get(key)
                if request['strategy_ref'] != strategy:
                    if cached is not None:
                        return copy.deepcopy(cached)
                    return phase_boundary(strategy)
                if cached is not None:
                    return copy.deepcopy(cached)
                if key not in discovery_order:
                    discovery_order[key] = len(discovery_order)
                outcome = evaluate_once(
                    request, search_pass=f'discovery:{strategy}',
                    planning_attempts=1, segment_time=discovery_time,
                    candidate_budget=discovery_candidate_budget)
                evaluation_cache[key] = copy.deepcopy(outcome)
                phase_discovered.append(key)
                if outcome.get('retryable'):
                    phase_retryable.add(key)
                return outcome

            resolution = resolve_task_intent(
                intent, environment, cell, targets, discover,
                now=resolution_reference_time, resolved=None)

            phase_record = {
                'strategy_ref': strategy,
                'phase_index': phase_index,
                'discovered_candidates': len(phase_discovered),
                'retryable_candidates': len(phase_retryable),
                'discovery_status': resolution['readiness_status'],
                'discovery_code': resolution['readiness']['primary_code'],
                'retry_priority': [],
                'retries': [],
            }
            phase_evidence.append(phase_record)

            if resolution['readiness_status'] in ('READY', 'WARNING'):
                break
            if resolution['readiness']['primary_code'] == 'SEARCH_BUDGET_EXHAUSTED':
                stop_all = True
                break
            if grasp_policy == 'EXACT':
                break

            retry_priority = sorted(
                phase_retryable,
                key=lambda key: (-evaluation_cache[key].get('progress_passes', 0),
                                 discovery_order.get(key, 1 << 30)))
            phase_record['retry_priority'] = [
                {'strategy_ref': key[0], 'object_id': key[1], 'candidate_id': key[2],
                 'progress_passes': evaluation_cache[key].get('progress_passes', 0),
                 'failed_stage': evaluation_cache[key].get('failed_stage')}
                for key in retry_priority
            ]

            for selected_key in retry_priority[:retry_beam_width]:
                if time.monotonic() >= deadline:
                    stop_all = True
                    break

                def retry(request, selected_key=selected_key, strategy=strategy):
                    key = request_key(request)
                    cached = evaluation_cache.get(key)
                    if key == selected_key:
                        retry_request = copy.deepcopy(request)
                        # Bind the exact approach IK branch that discovery
                        # already proved for this same candidate/scene. This is
                        # a seed/branch identity only; the retry still performs
                        # a fresh collision-aware MoveGroup plan from home.
                        if cached is not None and cached.get('approach_ik') is not None:
                            retry_request['approach_ik'] = copy.deepcopy(cached['approach_ik'])
                        if cached is not None and cached.get('extraction_intent') is not None:
                            retry_request['extraction_intent'] = copy.deepcopy(cached['extraction_intent'])
                            retry_request['transfer_ik_seed'] = copy.deepcopy(cached.get('transfer_ik_seed'))
                        outcome = evaluate_once(
                            retry_request, search_pass=f'retry:{strategy}',
                            planning_attempts=3, segment_time=float(planning_time),
                            candidate_budget=retry_candidate_budget)
                        if (outcome.get('approach_ik') is None and cached is not None and
                                cached.get('approach_ik') is not None):
                            outcome['approach_ik'] = copy.deepcopy(cached['approach_ik'])
                        evaluation_cache[key] = copy.deepcopy(outcome)
                        return outcome
                    if cached is not None:
                        return copy.deepcopy(cached)
                    return phase_boundary(strategy)

                resolution = resolve_task_intent(
                    intent, environment, cell, targets, retry,
                    now=resolution_reference_time, resolved=None)
                current = evaluation_cache.get(selected_key, {})
                attempt = {
                    'strategy_ref': selected_key[0],
                    'object_id': selected_key[1],
                    'candidate_id': selected_key[2],
                    'progress_passes': current.get('progress_passes', 0),
                    'failed_stage': current.get('failed_stage'),
                    'result_status': resolution['readiness_status'],
                    'result_code': resolution['readiness']['primary_code'],
                }
                phase_record['retries'].append(copy.deepcopy(attempt))
                retry_attempted.append(attempt)

                if resolution['readiness_status'] in ('READY', 'WARNING'):
                    break
                if resolution['readiness']['primary_code'] == 'SEARCH_BUDGET_EXHAUSTED':
                    stop_all = True
                    break

            if resolution['readiness_status'] in ('READY', 'WARNING') or stop_all:
                break

        summary['candidate_search'] = {
            'mode': 'strategy_phased_progress_beam',
            'strategy_phases': strategy_phases,
            'discovery_planning_attempts': 1,
            'discovery_planning_time': discovery_time,
            'discovery_candidate_wall_budget': discovery_candidate_budget,
            'retry_candidate_wall_budget': retry_candidate_budget,
            'retry_beam_width': retry_beam_width,
            'full_planning_time': float(planning_time),
            'phases': phase_evidence,
            'retry_attempted': retry_attempted,
            'retry_pass_used': bool(retry_attempted),
        }

        if resolution is None:
            raise RuntimeError('candidate strategy search produced no resolution')

    attempts = summary.get('candidate_attempts', [])
    summary['candidate_search'].update(
        fresh_target_objects=len(targets),
        objects_considered=len({a['object_id'] for a in attempts}),
        grasps_considered=len({(a['object_id'], a['candidate_id']) for a in attempts}),
        extraction_variants_evaluated=sum(len(a.get('extraction_attempts', [])) for a in attempts),
        selected_extraction=copy.deepcopy(resolution.get('grasp_resolution', {}).get('extraction_intent')))
    summary['task_intent_resolution'] = resolution
    summary['normalized_intent_sha256'] = resolution['normalized_intent_sha256']
    summary['resolution_sha256'] = resolution['resolution_sha256']
    if resolution['readiness_status'] not in ('READY', 'WARNING'):
        raise RuntimeError(resolution['readiness']['primary_code'] + ': ' + resolution['readiness']['reason'])
    grasp = resolution['grasp_resolution']
    cycle = cycles[grasp['selected_candidate_id'], grasp['selected_object_id']]
    cycle['grasp_index'] = int(grasp['selected_candidate_id'].rsplit('::', 1)[1])
    return cycle


def require_prevalidated_execution(start, cycle):
    if not start or not cycle.get('full_cycle_prevalidated'):
        raise RuntimeError('execution requires --start and a fully prevalidated cycle')


def updated_state(state, positions, mimics=()):
    result = copy.deepcopy(state)
    values = dict(zip(result.joint_state.name, result.joint_state.position))
    values.update(positions)
    for name, parent, multiplier, offset in mimics:
        values[name] = values[parent] * multiplier + offset
    result.joint_state.position = [float(values[n]) for n in result.joint_state.name]
    result.is_diff = False
    return result


def assert_joint_match(actual, expected, tolerance=0.005):
    values = dict(zip(actual.joint_state.name, actual.joint_state.position))
    for name, value in zip(expected.joint_state.name, expected.joint_state.position):
        if name not in values or not math.isfinite(values[name]) or abs(values[name] - value) > tolerance:
            raise RuntimeError(f'joint state diverged: {name}, expected={value}, actual={values.get(name)}')


def observation_geometry_matches(observed, actual):
    """Compare rigid geometry, including q == -q after MoveIt canonicalization."""
    if actual is None:
        return False
    a, b = observed['pose'], actual['pose']
    values = a + b + observed['dimensions'] + actual['dimensions']
    return (all(math.isfinite(v) for v in values)
            and all(abs(x-y) <= 1e-7 for x, y in zip(a[:3], b[:3]))
            and all(abs(x-y) <= 1e-7 for x, y in zip(observed['dimensions'], actual['dimensions']))
            and min(math.dist(a[3:], b[3:]), math.dist(a[3:], [-v for v in b[3:]])) <= 1e-7)


def observations_to_insert(objects,existing,backend):
    ids={o['id'] for o in objects}
    duplicates=ids.intersection(existing)
    if backend!='simulator' and duplicates:
        raise RuntimeError('runtime IDs already exist; reset the fake scene before replay')
    if backend=='simulator':
        if {k for k in existing if k.startswith('runtime::')}-ids:
            raise RuntimeError('unknown prior simulator runtime objects require measured reconciliation')
        if any(not observation_geometry_matches(o,existing[o['id']]) for o in objects if o['id'] in duplicates):
            raise RuntimeError('simulator observation geometry changed; refresh and reconcile')
    return [o for o in objects if o['id'] not in duplicates]


def wait_for_robot_baseline(read_scene, home, mimics=(), timeout=10.0):
    """Wait for initial state publication; never command home or relax tolerance."""
    required = dict(home, gripper_finger1_joint=0.0)
    deadline = time.monotonic() + timeout
    while True:
        scene = read_scene()
        try:
            if not set(required).issubset(scene.robot_state.joint_state.name):
                raise RuntimeError('required arm/gripper joint state not received')
            assert_joint_match(scene.robot_state,
                updated_state(scene.robot_state, required, mimics), 0.001)
            return scene
        except RuntimeError as exc:
            if time.monotonic() >= deadline:
                raise RuntimeError(f'canonical home/open-gripper state unavailable: {exc}') from exc
            time.sleep(0.1)


def private_attachment(scene, original, frame, frame_pose, touch_links):
    """Attach in a private predicted state using an explicit link-relative transform."""
    result = copy.deepcopy(scene)
    obj = copy.deepcopy(original)
    geometry = collision_object_dict(obj)
    relative = _PLANNER.compose_pose(_PLANNER.inverse_pose(pose_values(frame_pose)), geometry['pose'])
    from geometry_msgs.msg import Pose
    obj.pose = Pose()
    (obj.pose.position.x, obj.pose.position.y, obj.pose.position.z,
     obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w) = relative
    obj.primitive_poses = [Pose()]
    obj.primitive_poses[0].orientation.w = 1.0
    obj.header.frame_id = frame
    result.robot_state.attached_collision_objects = attachment_diff(obj, frame, touch_links, False).robot_state.attached_collision_objects
    result.robot_state.is_diff = False
    result.world.collision_objects = [o for o in result.world.collision_objects if o.id != obj.id]
    return result


def assert_scene_match(actual, expected, selected_id=None):
    """Reject changed obstacles, ACM, attachments or robot state before continuing."""
    assert_joint_match(actual.robot_state, expected.robot_state)
    if collision_matrix_signature(actual.allowed_collision_matrix) != collision_matrix_signature(expected.allowed_collision_matrix):
        raise RuntimeError('allowed collision matrix diverged')
    def geometry_equal(a, b):
        ga, gb = collision_object_dict(a), collision_object_dict(b)
        if a.header.frame_id != b.header.frame_id:
            return False
        if ga is not None and gb is not None:
            tol = 0.003 if a.id == selected_id else 1e-7
            qa, qb = ga['pose'][3:], gb['pose'][3:]
            return (all(abs(x-y) < 1e-7 for x,y in zip(ga['dimensions'], gb['dimensions']))
                    and math.dist(ga['pose'][:3], gb['pose'][:3]) <= tol
                    and min(math.dist(qa,qb), math.dist(qa,[-v for v in qb])) <= (0.005 if a.id == selected_id else 1e-7))
        a, b = copy.deepcopy(a), copy.deepcopy(b)
        a.header.stamp.sec = b.header.stamp.sec = 0
        a.header.stamp.nanosec = b.header.stamp.nanosec = 0
        # Compare ROS fields, not CDR bytes: transport alignment padding is
        # not geometry and may differ between equivalent messages.
        return a == b
    aw = {o.id:o for o in actual.world.collision_objects}
    ew = {o.id:o for o in expected.world.collision_objects}
    if set(aw) != set(ew) or any(not geometry_equal(aw[k],ew[k]) for k in ew):
        details = [dict(id=k,actual_pose=pose_values(aw[k].pose),expected_pose=pose_values(ew[k].pose),
                        actual_frame=aw[k].header.frame_id,expected_frame=ew[k].header.frame_id)
                   for k in set(aw).intersection(ew) if not geometry_equal(aw[k],ew[k])]
        raise RuntimeError(f'world collision geometry diverged: missing={set(ew)-set(aw)}, extra={set(aw)-set(ew)}, changed={details}')
    aa = {o.object.id:o for o in actual.robot_state.attached_collision_objects}
    ea = {o.object.id:o for o in expected.robot_state.attached_collision_objects}
    if set(aa) != set(ea) or any(aa[k].link_name != ea[k].link_name or
            set(aa[k].touch_links) != set(ea[k].touch_links) or
            not geometry_equal(aa[k].object,ea[k].object) for k in ea):
        raise RuntimeError('attached object state diverged')
    if (actual.world.octomap != expected.world.octomap or
            actual.link_padding != expected.link_padding or actual.link_scale != expected.link_scale):
        raise RuntimeError('collision map/padding/scale diverged')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--summary-output', required=True)
    parser.add_argument('--planning-trace', type=Path, help='Opt-in complete IK/request/private-scene evidence directory')
    parser.add_argument('--scene-package', default='ur5_2f_test')
    parser.add_argument('--task-request')
    parser.add_argument('--resolve-task', action='store_true', help='Resolve the saved authored task and write derived evidence after plan-only verification')
    parser.add_argument('--detections', required=True)
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--start', action='store_true')
    parser.add_argument('--backend', choices=('fake', 'simulator'), default='fake')
    parser.add_argument('--simulator-commission', choices=('cancel', 'telemetry', 'stationary', 'contact-release', 'full-cycle'), help='Explicit bounded simulator trial; ordinary execution remains blocked')
    parser.add_argument('--commission-evidence', type=Path, help='Passing measured trial and cancellation evidence required for full-cycle')
    parser.add_argument('--simulator-receipt', type=Path, help='Live local Fortress receipt; independently verified, never an identity bypass')
    parser.add_argument('--segment-planning-time', type=float, default=3.0, help='Per-request computation budget, 0 < seconds <= 10; collision tolerances unchanged')
    parser.add_argument('--timeout', type=float, default=180.0, help='Total candidate search budget in seconds')
    parser.add_argument('--retreat-distance', type=float)
    args = parser.parse_args()
    if not math.isfinite(args.segment_planning_time) or not 0 < args.segment_planning_time <= 10:
        parser.error('segment planning time must be finite and in (0, 10]')
    if args.simulator_commission and (not args.start or args.resolve_task):
        parser.error('commissioning requires --start and a consumed generated resolution; resolve separately')
    import yaml
    import xml.etree.ElementTree as ET
    import rclpy
    from rclpy.action import ActionClient
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents, Constraints,
        JointConstraint, MotionPlanRequest, PositionConstraint, OrientationConstraint,
        BoundingVolume)
    from shape_msgs.msg import SolidPrimitive
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListHardwareComponents, ListControllers, ListHardwareInterfaces
    spec = importlib.util.spec_from_file_location('runtime_pick_inputs', Path(__file__).with_name('runtime_pick_inputs.py'))
    inputs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(inputs)
    from rclpy.signals import SignalHandlerOptions
    # Keep the context alive until bounded cancellation/recovery has finished.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'interrupted by signal {signum}')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = rclpy.create_node('perceived_object_grasp_execute')
    summary = dict(result='FAIL', execution_attempted=False, full_cycle_prevalidated=False,
                   full_cycle_execution_success=False, stages=[], candidate_attempts=[])
    def stage(name):
        if summary.get('current_stage') == name:
            return
        summary['current_stage'] = name
        summary.setdefault('stage_events', []).append(dict(stage=name, timestamp=time.time()))
        print(json.dumps({'stage':name}), flush=True)
    stage('IDLE')
    scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    apply_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    fk_client = node.create_client(GetPositionFK, '/compute_fk')
    ik_client = node.create_client(GetPositionIK, '/compute_ik')
    validity_client = node.create_client(GetStateValidity, '/check_state_validity')
    params_client = node.create_client(GetParameters, '/move_group/get_parameters')
    hardware_client = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
    plan_client = ActionClient(node, MoveGroup, '/move_action')
    execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
    selected_id = None
    measurements = None
    contact_guard = None
    execution_monitor = None
    controlled_cancel = False
    controller_audit = None
    trace_sequence = 0
    def trace(label, message):
        nonlocal trace_sequence
        if args.planning_trace is None:
            return
        from rosidl_runtime_py.convert import message_to_ordereddict
        args.planning_trace.mkdir(parents=True, exist_ok=True)
        record = dict(wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
                      stage=summary.get('current_stage'),
                      candidate_attempt=len(summary['candidate_attempts']),
                      message=message_to_ordereddict(message))
        (args.planning_trace/f'{trace_sequence:05d}-{label}.json').write_text(json.dumps(record, indent=2)+'\n')
        trace_sequence += 1
    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'service timeout: {client.srv_name}')
        return future.result()
    def scene_now():
        req = GetPlanningScene.Request()
        req.components.components = 1023
        return call(scene_client, req).scene
    def apply(diff):
        if not call(apply_client, ApplyPlanningScene.Request(scene=diff)).success:
            raise RuntimeError('PlanningScene rejected transition')
    def action(client, goal, timeout):
        if client is execute_client:
            from rosidl_runtime_py.convert import message_to_ordereddict
            trajectory_evidence=message_to_ordereddict(goal.trajectory)
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt action unavailable')
        if controlled_cancel and controller_audit:controller_audit.arm()
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5)
        if not sent.done():
            # A late accepted goal must not keep running after the caller fails.
            sent.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result() and f.result().accepted else None)
            raise RuntimeError('action acceptance timed out')
        handle = sent.result()
        if not handle or not handle.accepted:
            raise RuntimeError('action goal rejected')
        future = handle.get_result_async()
        owned_uuid=list(handle.goal_id.uuid)
        motion_trial=None
        if client is execute_client:
            summary['owned_execution_goal']=dict(uuid=bytes(owned_uuid).hex(),accepted=True,
                stage=summary.get('current_stage'),wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),
                trajectory=trajectory_evidence)
        try:
            cancel_start=measurements.fresh()['sim_ns'] if measurements else 0
            if controlled_cancel:
                from simulator_execution import CancellationMotion
                if not set(contract['home_joint_names']).issubset(goal.trajectory.joint_trajectory.joint_names):
                    raise RuntimeError('cancellation approach does not command the complete arm')
                accepted_sample=measurements.fresh()
                motion_trial=CancellationMotion(bytes(owned_uuid).hex(),accepted_sample,
                    measurements.joints(accepted_sample),contract['home_joint_names'],
                    accepted_wall_ns=summary['owned_execution_goal']['wall_ns'])
                summary['cancellation_motion']=motion_trial.evidence
            if execution_monitor is None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            else:
                until=time.monotonic()+timeout
                while not future.done() and time.monotonic()<until:
                    rclpy.spin_once(node,timeout_sec=.005)
                    execution_monitor()
                    if controlled_cancel:
                        sample=measurements.fresh()
                        moved=motion_trial.observe(sample,measurements.joints(sample))
                        summary['cancellation_movement_verified']=moved
                        if moved and sample['sim_ns']-cancel_start>200000000:
                            raise RuntimeError('CONTROLLED_CANCELLATION')
            if not future.done() or future.result() is None:
                raise RuntimeError('action result timed out')
            if controlled_cancel:
                # A short/successful approach must never turn this bounded trial
                # into contact motion when cancellation was not demonstrated.
                raise RuntimeError('CANCELLATION_TRIAL_ENDED_BEFORE_CANCEL')
        except BaseException:
            if measurements:
                from simulator_execution import cancel_owned,cancel_response_matches
                def request_cancel():
                    summary['cancellation_request']=dict(uuid=bytes(owned_uuid).hex(),
                        wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns())
                    try:
                        summary['cancel_measurement']=measurements.fresh()
                    except RuntimeError as exc:
                        # Lost evidence must fail acceptance, never prevent the
                        # owned stop request that makes the failure safe.
                        summary['cancel_measurement_error']=str(exc)
                    if controlled_cancel and controller_audit:
                        try:controller_audit.before_cancel()
                        except Exception as exc:summary['controller_audit_failure']=str(exc)
                    summary['cancellation_request'].update(wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns())
                    cancel=handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node,cancel,timeout_sec=5)
                    response=cancel.result() if cancel.done() else None
                    accepted=cancel_response_matches(owned_uuid,response)
                    summary['cancellation_accepted']=accepted
                    summary['cancellation_response']=dict(return_code=response.return_code if response else None,
                        goal_uuids=[bytes(g.goal_id.uuid).hex() for g in response.goals_canceling] if response else [],
                        wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns())
                    return accepted if controlled_cancel else accepted or future.done()
                def terminal():
                    rclpy.spin_until_future_complete(node,future,timeout_sec=5)
                    ended=bool(future.done() and future.result() and future.result().status in (4,5,6))
                    summary['interrupted_action_terminal_status']=future.result().status if ended else None
                    summary['interrupted_action_terminal']=dict(uuid=bytes(owned_uuid).hex(),
                        status=summary['interrupted_action_terminal_status'],
                        moveit_code=future.result().result.error_code.val if ended else None,
                        wall_ns=time.time_ns(),monotonic_ns=time.monotonic_ns())
                    controller_ok=True
                    if controlled_cancel and controller_audit:
                        try:summary['controller_cancellation']=controller_audit.collect()
                        except Exception as exc:
                            controller_ok=False;summary['controller_audit_failure']=str(exc)
                        summary['controller_status_events']=controller_audit.events
                    return ended and controller_ok and (not controlled_cancel or future.result().status==5)
                def stopped():
                    summary['motion_stop_verified']=wait_stopped()
                    return summary['motion_stop_verified']
                summary['cancellation_confirmed']=False
                try:
                    cancel_owned(request_cancel,terminal,stopped,
                        lambda:apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline)),measured_reconcile)
                    summary['cancellation_confirmed']=True
                except Exception as cancel_error:summary['cancellation_failure']=str(cancel_error)
                raise
            summary['cancellation_confirmed'] = False
            if rclpy.ok():
                try:
                    cancel = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
                    summary['cancellation_accepted'] = bool(
                        cancel.done() and cancel.result() and cancel.result().goals_canceling)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                    summary['cancellation_confirmed'] = bool(
                        future.done() and future.result() and future.result().status == 5)
                    if future.done() and future.result():
                        summary['interrupted_action_terminal_status'] = future.result().status
                    if measurements:
                        summary['motion_stop_verified']=wait_stopped()
                        summary['cancellation_confirmed'] = summary['cancellation_confirmed'] and summary['motion_stop_verified']
                except Exception as cancel_error:
                    summary['cancellation_failure'] = str(cancel_error)
            else:
                summary['cancellation_failure'] = 'ROS context already invalid'
            raise
        response = future.result()
        if response.status != 4 or response.result.error_code.val != 1:
            raise MoveItActionFailure(response.status, response.result.error_code.val)
        return response.result
    def wait_stopped(monitor_contacts=False):
        from simulator_execution import StopWindow
        names={j.get('name') for control in measurements.robot.findall('ros2_control') for j in control.findall('joint')}
        window=StopWindow(names)
        travel_samples=[]
        measurements.fresh()
        after_terminal_wall_ns=time.time_ns()
        until=time.monotonic()+8
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            measurements.fresh()
            pending=measurements.drain()
            stopped=False;stop_sample=None
            for sample in pending:
                if monitor_contacts:contact_guard.check(sample)
                measurements.record('stopping',sample)
                if sample['wall_ns']>=summary.get('cancel_measurement',sample)['wall_ns']:travel_samples.append(sample)
                if sample['wall_ns']<after_terminal_wall_ns:continue
                stopped=window.observe(sample,measurements.joints(sample))
                stop_sample=sample
            if stopped and 0<=time.time()-stop_sample['wall_ns']/1e9<=.25:
                measurements.fresh()
                summary['stopped_measurement']=stop_sample
                summary['stopped_window']=window.evidence
                if summary.get('cancellation_motion'):
                    from simulator_execution import cancellation_metrics
                    summary['cancellation_metrics']=cancellation_metrics(summary,measurements,travel_samples)
                return True
        return False
    def measured_fcl():
        from simulator_execution import validate_measured_contacts,measured_attachment
        s=measurements.fresh()
        state=copy.deepcopy(initial.robot_state)
        measured=measurements.joints(s)
        state=updated_state(state,{k:v[0] for k,v in measured.items()}, {})
        carried=contact_guard.planning_attached and contact_guard.phase!='released'
        # The state-validity service otherwise compares measured robot joints
        # against the old observation world. Keep every physical BOX at the
        # same authoritative measurement used for this query.
        world_diff=PlanningScene(is_diff=True)
        for original in (initial.world.collision_objects if contact_guard.held else []):
            if original.id not in contact_guard.pile_objects or (carried and original.id==selected_id):continue
            obj=copy.deepcopy(original)
            values=measurements.object_pose(s,contact_guard.pile_objects[obj.id]['name'])
            (obj.pose.position.x,obj.pose.position.y,obj.pose.position.z,
             obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w)=values
            world_diff.world.collision_objects.append(obj)
        if world_diff.world.collision_objects:apply(world_diff)
        if carried:
            original=next(o for o in initial.world.collision_objects if o.id==selected_id)
            state.attached_collision_objects=measured_attachment(original,contract,measurements,s).robot_state.attached_collision_objects
        else:state.attached_collision_objects=[]
        state.is_diff=False
        future=validity_client.call_async(GetStateValidity.Request(robot_state=state,group_name=''))
        rclpy.spin_until_future_complete(node,future,timeout_sec=.2)
        if not future.done() or not future.result():raise RuntimeError('measured collision query timed out')
        try:
            validate_measured_contacts(future.result(),contact_guard.support if contact_guard.held else None,
                bool(contact_guard.separation and contact_guard.separation.expired),contact_guard.predicate,
                pile_guard=contact_guard if carried else None)
        except RuntimeError:
            from rosidl_runtime_py.convert import message_to_ordereddict
            # Preserve the exact rejected query before cancellation changes the
            # physical state or reconciliation changes the planning scene.
            summary.setdefault('rejected_measured_collision_check', dict(
                measurement=copy.deepcopy(s), robot_state=message_to_ordereddict(state),
                response=message_to_ordereddict(future.result())))
            raise
        measurements.fresh()
        summary['last_measured_collision_check']=dict(sim_ns=s['sim_ns'],valid=future.result().valid,contacts=len(future.result().contacts))
    def measured_reconcile():
        # Revoke first. Never restore predicted pre-motion poses.
        apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
        s=measurements.fresh(); p=measurements.object_pose(s,contact_guard.name)
        original=next(o for o in initial.world.collision_objects if o.id==selected_id)
        held=False
        if contact_guard.held and contact_guard.phase not in ('opening','released'):
            try:
                s=contact_guard.checked_current();p=measurements.object_pose(s,contact_guard.name);held=True
            except RuntimeError:pass
        if held:
            from simulator_execution import measured_attachment
            reconciliation=measured_attachment(original,contract,measurements,s)
            apply(reconciliation)
        else:
            reconciliation=place_detachment_diff(original,contract['grasp_frame'],p[:3],p[3:])
            apply(reconciliation)
        current=scene_now()
        measured_objects=([a.object for a in current.robot_state.attached_collision_objects]
                          if held else current.world.collision_objects)
        expected_object=(reconciliation.robot_state.attached_collision_objects[0].object if held
                         else reconciliation.world.collision_objects[0])
        actual_objects=[o for o in measured_objects if o.id==selected_id]
        geometry_matches=(len(actual_objects)==1 and observation_geometry_matches(
            collision_object_dict(expected_object),collision_object_dict(actual_objects[0])))
        summary['measured_reconciliation']=dict(held=held,pose=p,sim_ns=s['sim_ns'],
            acm_restored=collision_matrix_signature(current.allowed_collision_matrix)==collision_matrix_signature(baseline),
            measured_geometry_matches=geometry_matches,
            attached_ids=[o.object.id for o in current.robot_state.attached_collision_objects])
        evidence_scene('planning_scene_reconciled',current)
        if not geometry_matches or not summary['measured_reconciliation']['acm_restored']:
            raise RuntimeError('measured scene reconciliation did not preserve geometry/ACM')
    def monitored_hold(seconds):
        start=measurements.fresh()['sim_ns'];until=time.monotonic()+max(10,seconds*20)
        while measurements.fresh()['sim_ns']-start<seconds*1e9:
            if time.monotonic()>until:raise RuntimeError('measurement hold timed out')
            rclpy.spin_once(node,timeout_sec=.005);contact_guard.drain()

    def fk(state, link):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = [link]
        req.robot_state = state
        result = call(fk_client, req)
        if result.error_code.val != 1 or len(result.pose_stamped) != 1:
            raise RuntimeError(f'FK failed: {result.error_code.val}')
        return result.pose_stamped[0]
    def pose_message(values):
        p = PoseStamped()
        p.header.frame_id = 'world'
        (p.pose.position.x,p.pose.position.y,p.pose.position.z,
         p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w) = values
        return p
    def joint_constraints(values):
        return Constraints(joint_constraints=[JointConstraint(joint_name=n, position=float(v),
            tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0) for n,v in values.items()])

    def cartesian_corridor_constraints(start, goal, frame_id, link_name, initial_support=None,
                                       position_tolerance=0.0025, orientation_tolerance=0.005):
        """Constrain OMPL itself to the same Cartesian tube verified afterwards."""
        direction = [b-a for a, b in zip(start[:3], goal[:3])]
        length = math.sqrt(sum(value*value for value in direction))
        if length < 1e-12:
            axis = [0.0, 0.0, 1.0]
        else:
            axis = [value/length for value in direction]
        # Quaternion rotating +Z onto the corridor axis.
        dot = max(-1.0, min(1.0, axis[2]))
        if dot > 1.0 - 1e-12:
            rotation = [0.0, 0.0, 0.0, 1.0]
        elif dot < -1.0 + 1e-12:
            rotation = [1.0, 0.0, 0.0, 0.0]
        else:
            raw = [-axis[1], axis[0], 0.0, 1.0 + dot]
            norm = math.sqrt(sum(value*value for value in raw))
            rotation = [value/norm for value in raw]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [length + 2.0*position_tolerance, position_tolerance]
        region_pose = Pose()
        region_pose.position.x = (start[0] + goal[0]) / 2.0
        region_pose.position.y = (start[1] + goal[1]) / 2.0
        region_pose.position.z = (start[2] + goal[2]) / 2.0
        (region_pose.orientation.x, region_pose.orientation.y,
         region_pose.orientation.z, region_pose.orientation.w) = rotation

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = link_name
        region = BoundingVolume()
        region.primitives = [primitive]
        region.primitive_poses = [region_pose]
        position.constraint_region = region
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame_id
        orientation.link_name = link_name
        (orientation.orientation.x, orientation.orientation.y,
         orientation.orientation.z, orientation.orientation.w) = goal[3:]
        # Quaternion Euclidean error 0.005 is approximately 0.01 rad.
        angular_tolerance = 2.0 * orientation_tolerance
        orientation.absolute_x_axis_tolerance = angular_tolerance
        orientation.absolute_y_axis_tolerance = angular_tolerance
        orientation.absolute_z_axis_tolerance = angular_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        if initial_support is not None:
            constraints.name = 'workcell_initial_support_contact:' + json.dumps(
                initial_support, sort_keys=True)
        return constraints

    def plan_segment(view, name, goal, group=None, straight=False, initial_support=None,
                     ik_binding=None, ik_seed=None, cartesian_corridor=None,
                     initial_separation_object_ids=None, extraction_intent=None):
        stage(name)
        if ik_seed is not None and (name != 'PREPLAN_TRANSFER' or isinstance(goal, dict)
                                   or (group is not None and group != contract['planning_group'])
                                   or ik_binding is not None or straight or cartesian_corridor is not None):
            raise RuntimeError('TRANSFER_IK_SEED_CONTEXT: seed requires an unbound transfer pose')
        wall_deadline = min(deadline, float(contract.get('_candidate_wall_deadline', deadline)))
        def wall_budget_failure():
            from full_cycle_preplanner import SearchBudgetExhausted, CandidateBudgetExhausted
            if time.monotonic() >= deadline:
                return SearchBudgetExhausted('candidate search budget exhausted')
            return CandidateBudgetExhausted('candidate wall-clock slice exhausted')
        if time.monotonic() > wall_deadline:
            raise wall_budget_failure()
        planning_attempts = int(contract.get('_candidate_planning_attempts', 3))
        planning_time = float(contract.get('_candidate_planning_time', args.segment_planning_time))
        if planning_attempts < 1 or planning_attempts > 3:
            raise RuntimeError('candidate planning attempts must be in [1, 3]')
        if not math.isfinite(planning_time) or planning_time <= 0:
            raise RuntimeError('candidate planning time must be finite and positive')
        remaining = wall_deadline - time.monotonic()
        if remaining <= 0:
            raise wall_budget_failure()
        planning_time = min(planning_time, args.segment_planning_time, remaining)
        before = copy.deepcopy(view)
        # Physical initial pile separation needs a conservative dynamic profile:
        # measured tracking and grasp slip can consume micron-scale side gaps.
        # Keep the same Cartesian path, collision policy and execution deadline.
        motion_scaling = .02 if getattr(args, 'backend', 'fake') == 'simulator' and name == 'PREPLAN_LIFT' else .2
        request = MotionPlanRequest(group_name=group or contract['planning_group'],
            start_state=copy.deepcopy(view.robot_state), num_planning_attempts=1,
            allowed_planning_time=planning_time, max_velocity_scaling_factor=motion_scaling,
            max_acceleration_scaling_factor=motion_scaling)
        request.start_state.is_diff = False
        if isinstance(goal, dict):
            request.goal_constraints = [joint_constraints(goal)]
            goal_state = updated_state(view.robot_state, goal, mimics)
        if straight:
            # Constrain one MoveGroup request to the full Cartesian tube, then
            # independently audit the returned trajectory with dense FK samples.
            start_pose = fk(view.robot_state, contract['tool_link'])
            a, b = pose_values(start_pose.pose), pose_values(goal.pose)
            count = max(1, math.ceil(math.dist(a[:3], b[:3]) / 0.005))
            support = None
            initial_separation_ids = []
            extraction_geometry = None
            extraction_poses = []
            if extraction_intent is not None:
                if name != 'PREPLAN_LIFT' or len(view.robot_state.attached_collision_objects) != 1:
                    raise RuntimeError('extraction intent requires one attached target at lift')
                object_id = view.robot_state.attached_collision_objects[0].object.id
                offset = extraction_intent.get('offset_xyz_m', [])
                if (extraction_intent.get('object_id') != object_id or len(offset) != 3 or
                        any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in offset) or
                        math.dist([b[i]-a[i] for i in range(3)], offset) > 1e-9):
                    raise RuntimeError('extraction intent disagrees with current Cartesian goal')
                original = next(o for o in initial.world.collision_objects if o.id == object_id)
                target_geometry = collision_object_dict(original)
                neighbor_geometry = [item for obj in view.world.collision_objects
                                     if obj.id != object_id and (item := collision_object_dict(obj)) is not None]
                extraction_geometry = (target_geometry, neighbor_geometry)
                from pile_extraction import audit_extraction
                # Geometry predicts suitability; it cannot create a physical
                # pile certificate or permit a new collision in MoveIt.
                audit_extraction(*extraction_geometry, extraction_intent)
                object_in_tool = _PLANNER.compose_pose(_PLANNER.inverse_pose(a), target_geometry['pose'])
            if name == 'PREPLAN_LIFT' and len(view.robot_state.attached_collision_objects) == 1:
                object_id = view.robot_state.attached_collision_objects[0].object.id
                original = next(o for o in initial.world.collision_objects if o.id == object_id)
                eligible_supports = legitimate_support_ids(collision_object_dict(original), cell['environment'], manifest)
                measured = call(validity_client, GetStateValidity.Request(robot_state=view.robot_state, group_name=''))
                contacts = contacts_in_planned_scene(measured.contacts, view, initial)
                perceived_ids = {item['id'] for item in summary.get('normalized_objects', [])}
                floor_contacts = []
                for c in contacts:
                    if c.contact_body_1 == object_id and c.body_type_1 == 2 and c.contact_body_2 in eligible_supports and c.normal.z < -.999999:
                        floor_contacts.append((c.contact_body_2, c.position.z))
                        continue
                    if c.contact_body_2 == object_id and c.body_type_2 == 2 and c.contact_body_1 in eligible_supports and c.normal.z > .999999:
                        floor_contacts.append((c.contact_body_1, c.position.z))
                        continue

                    neighbor = None
                    if (c.contact_body_1 == object_id and c.body_type_1 == 2 and
                            c.body_type_2 == 1 and c.contact_body_2 in perceived_ids):
                        neighbor = c.contact_body_2
                    elif (c.contact_body_2 == object_id and c.body_type_2 == 2 and
                            c.body_type_1 == 1 and c.contact_body_1 in perceived_ids):
                        neighbor = c.contact_body_1
                    if neighbor is not None:
                        if (not math.isfinite(c.depth) or c.depth < 0.0 or
                                c.depth > 0.0001):
                            from full_cycle_preplanner import MotionFeasibilityFailure
                            raise MotionFeasibilityFailure(
                                'initial piled-object contact exceeds 0.1 mm numerical separation tolerance',
                                contacts=[c])
                        initial_separation_ids.append(neighbor)
                initial_separation_ids = sorted(set(initial_separation_ids))
                if floor_contacts and len({p[0] for p in floor_contacts}) == 1:
                    if 'workcell/InitialSupportContact' not in support_adapters.split():
                        raise RuntimeError('SUPPORT_CONTACT_ADAPTER_MISSING: regenerate and launch the current MoveIt configuration')
                    support = dict(object_id=object_id, support_id=floor_contacts[0][0],
                                   floor_z=floor_contacts[0][1], tool_link=contract['tool_link'])
            # Path constraints now encode the complete Cartesian tube, so one
            # constrained MoveGroup request is both stronger and dramatically
            # cheaper than chaining ~20 independent 5 mm OMPL requests. Keep an
            # independent dense FK audit over the returned trajectory as a
            # second guard; the planner constraint is never treated as proof by
            # itself.
            part = plan_segment(
                view, name, goal, group,
                initial_support=support,
                cartesian_corridor=(a, b),
                initial_separation_object_ids=initial_separation_ids)
            trajectory = part['trajectory']
            names = list(trajectory.joint_trajectory.joint_names)
            points = trajectory.joint_trajectory.points
            if not points:
                raise RuntimeError('Cartesian-constrained plan returned empty trajectory')

            previous_positions = dict(zip(names, points[0].positions))
            samples_checked = 0
            for point_index, point in enumerate(points):
                current_positions = dict(zip(names, point.positions))
                if point_index == 0:
                    fractions = (1.0,)
                else:
                    # Densify the post-plan audit in joint space. 0.02 rad is
                    # intentionally conservative for the short contact/lift/
                    # retreat motions while avoiding extra planning requests.
                    max_delta = max(
                        (abs(current_positions[n] - previous_positions[n]) for n in names),
                        default=0.0)
                    subdivisions = max(1, math.ceil(max_delta / 0.02))
                    fractions = tuple(i / subdivisions for i in range(1, subdivisions + 1))
                for fraction in fractions:
                    if point_index == 0:
                        positions = current_positions
                    else:
                        positions = {
                            n: previous_positions[n] +
                               (current_positions[n] - previous_positions[n]) * fraction
                            for n in names
                        }
                    sample = updated_state(view.robot_state, positions, mimics)
                    actual_pose = pose_values(fk(sample, contract['tool_link']).pose)
                    samples_checked += 1
                    if extraction_geometry is not None:
                        extraction_poses.append(_PLANNER.compose_pose(actual_pose, object_in_tool))
                    if not pose_within_cartesian_corridor(actual_pose, a, b):
                        from full_cycle_preplanner import MotionFeasibilityFailure
                        failure = MotionFeasibilityFailure(
                            'planned contact/retreat path leaves the Cartesian corridor')
                        failure.details.update(
                            path_pose=actual_pose, corridor_start=a, corridor_goal=b,
                            trajectory_point_index=point_index,
                            interpolation_fraction=fraction)
                        raise failure
                previous_positions = current_positions

            metadata = copy.deepcopy(part['metadata'])
            if extraction_geometry is not None:
                metadata['extraction_geometry_audit'] = audit_extraction(
                    *extraction_geometry, extraction_intent, poses=extraction_poses)
                metadata['extraction_intent'] = copy.deepcopy(extraction_intent)
            metadata.update(
                stage=name,
                success=True,
                moveit_code=1,
                points=len(points),
                cartesian_waypoints=1,
                cartesian_validation_segments=count,
                cartesian_validation_samples=samples_checked,
                initial_support_contact=support,
                initial_separation_object_ids=initial_separation_ids,
                attached_ids=[o.object.id for o in before.robot_state.attached_collision_objects],
                world_ids=[o.id for o in before.world.collision_objects])
            return dict(
                kind='motion', stage=name, before=before,
                after=copy.deepcopy(part['after']), trajectory=trajectory,
                metadata=metadata)
        if not isinstance(goal, dict):
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = contract['planning_group']
            ik_request.ik_request.ik_link_name = contract['tool_link']
            ik_request.ik_request.pose_stamped = goal
            ik_request.ik_request.robot_state = (bound_approach_seed(ik_binding, goal, contract, view.robot_state, mimics)
                                                if ik_binding is not None else copy.deepcopy(view.robot_state))
            if ik_seed is not None:
                ik_request.ik_request.robot_state = bound_transfer_seed(
                    ik_seed, goal, contract, view.robot_state, mimics)
            # Avoid rejecting a pose solely because IK selected a colliding arm
            # branch. The live service is usable only while its collision scene
            # matches this private view; otherwise preserve seed continuity and
            # let the private MoveGroup plan establish feasibility. Neither IK
            # outcome replaces the complete candidate motion checks.
            ik_request.ik_request.avoid_collisions = ik_scene_matches_live(view, initial)
            ik_request.ik_request.timeout.sec = 1
            trace('ik-request', ik_request)
            ik = call(ik_client, ik_request)
            trace('ik-response', ik)
            if ik.error_code.val != 1:
                from full_cycle_preplanner import MotionFeasibilityFailure
                raise MotionFeasibilityFailure(f'IK failed: {ik.error_code.val}', moveit_code=ik.error_code.val)
            values = dict(zip(ik.solution.joint_state.name,ik.solution.joint_state.position))
            if ik_binding is not None:
                # A changed collision scene may reject the saved branch. Never
                # silently substitute a new random branch under a READY handoff.
                if any(n not in values or not math.isfinite(values[n]) or abs(values[n]-v) > 0.0001
                       for n,v in ik_binding['joint_positions'].items()):
                    raise RuntimeError('APPROACH_IK_BRANCH_CHANGED: resolve current scene again')
            if ik_seed is not None:
                # Revalidate the proven transfer branch without silently
                # substituting another solution under the saved candidate.
                if any(n not in values or not math.isfinite(values[n]) or abs(values[n]-v) > 0.0001
                       for n,v in ik_seed['joint_positions'].items()):
                    raise RuntimeError('TRANSFER_IK_BRANCH_CHANGED: resolve current scene again')
            request.goal_constraints = [joint_constraints({n:values[n] for n in contract['home_joint_names']})]
            goal_state = updated_state(view.robot_state,
                                       {n: values[n] for n in contract['home_joint_names']}, mimics)
        if cartesian_corridor is not None:
            if 'workcell/StraightCartesianPath' not in support_adapters.split():
                raise RuntimeError(
                    'CARTESIAN_PATH_ADAPTER_MISSING: rebuild and launch the current MoveIt configuration')
            request.path_constraints = cartesian_corridor_constraints(
                cartesian_corridor[0], cartesian_corridor[1],
                goal.header.frame_id or 'world', contract['tool_link'], initial_support)
            marker = Constraints()
            marker.name = 'workcell_cartesian_path:' + json.dumps({
                'schema': 'workcell_cartesian_path/v1',
                'stage': name,
                'tool_link': contract['tool_link'],
                'start_pose': list(cartesian_corridor[0]),
                'goal_pose': list(cartesian_corridor[1]),
                'max_step_m': 0.0025,
                'initial_separation_object_ids': list(initial_separation_object_ids or []),
                'allow_initial_attached_world_separation':
                    bool(initial_separation_object_ids) and name == 'PREPLAN_LIFT',
            }, sort_keys=True, separators=(',', ':'))
            request.trajectory_constraints.constraints = [marker]
        elif initial_support is not None:
            request.path_constraints.name = 'workcell_initial_support_contact:' + json.dumps(initial_support, sort_keys=True)
        goal_msg = MoveGroup.Goal(request=request)
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.planning_scene_diff = copy.deepcopy(view)
        trace('move-group-goal', goal_msg)
        # OMPL can return an invalid sampled path or exhaust one stochastic
        # planning window for a feasible fixed goal. Retry that identical
        # plan-only request, never a different candidate, target, policy,
        # start state, scene or execution action. All collision checks remain active.
        for attempt in range(planning_attempts):
            try:
                result = action(plan_client, goal_msg, 12)
                break
            except MoveItActionFailure as exc:
                if time.monotonic() >= wall_deadline:
                    raise wall_budget_failure() from exc
                if (not retryable_plan_failure(exc.code) or
                        attempt == planning_attempts - 1):
                    from full_cycle_preplanner import MotionFeasibilityFailure
                    # For a failed short Cartesian segment this is its first
                    # rejected waypoint. No invalid state is applied or executed.
                    try:
                        validity = call(validity_client, GetStateValidity.Request(robot_state=goal_state, group_name=''))
                        contacts = contacts_in_planned_scene(validity.contacts, view, initial)
                    except Exception:
                        contacts = []  # Unavailable collision evidence is not a collision claim.
                    raise MotionFeasibilityFailure(str(exc), moveit_code=exc.code, contacts=contacts) from exc
                summary.setdefault('planning_retries', []).append(
                    {'stage': name, 'moveit_code': exc.code, 'attempt': attempt + 1,
                     'retry_kind': 'timed_out' if exc.code == -6 else 'invalid_motion_plan',
                     'search_pass': contract.get('_candidate_search_pass', 'normal')})
        trajectory = result.planned_trajectory
        if not trajectory.joint_trajectory.points:
            raise RuntimeError('MoveIt returned empty trajectory')
        # Reject start-state adapters silently moving the start out of collision.
        assert_joint_match(result.trajectory_start, before.robot_state, 0.001)
        first = updated_state(before.robot_state, dict(zip(trajectory.joint_trajectory.joint_names,
                    trajectory.joint_trajectory.points[0].positions)), mimics)
        assert_joint_match(first, before.robot_state, 0.001)
        after = copy.deepcopy(view)
        after.robot_state = updated_state(view.robot_state, dict(zip(trajectory.joint_trajectory.joint_names,
                    trajectory.joint_trajectory.points[-1].positions)), mimics)
        return dict(kind='motion', stage=name, before=before, after=after, trajectory=trajectory,
                    metadata=dict(stage=name, success=True, moveit_code=result.error_code.val,
                        planning_time=result.planning_time, points=len(trajectory.joint_trajectory.points),
                        allowed_planning_time=planning_time, planning_attempts=planning_attempts,
                        max_velocity_scaling_factor=motion_scaling, max_acceleration_scaling_factor=motion_scaling,
                        **({'cartesian_planner':'workcell/StraightCartesianPath'}
                           if cartesian_corridor is not None else {}),
                        **({'approach_ik': copy.deepcopy(ik_binding) if ik_binding is not None else approach_ik_binding(goal, contract, values)}
                           if name == 'PREPLAN_APPROACH' and not isinstance(goal, dict) else {}),
                        **({'transfer_ik_seed': copy.deepcopy(ik_seed) if ik_seed is not None else transfer_ik_seed(goal, contract, values)}
                           if name == 'PREPLAN_TRANSFER' and not isinstance(goal, dict) else {}),
                        attached_ids=[o.object.id for o in view.robot_state.attached_collision_objects],
                        world_ids=[o.id for o in view.world.collision_objects]))
    try:
        stage('PREFLIGHT')
        package = Path(args.scene_package)
        if not package.is_dir():
            package = Path(get_package_share_directory(args.scene_package))
        contract = _PLANNER.load_grasp_contract(package)
        cell = yaml.safe_load((package/'cell_definition.yaml').read_text())
        authored = args.resolve_task or cell.get('builder_task_intent', {}).get('schema') == 'workcell_builder_task_intent/v2'
        if args.backend == 'simulator' and (not authored or args.replay):
            raise RuntimeError('simulator requires current authored TaskIntent and timestamped physical observations, not replay')
        intent = None
        if authored:
            from task_intent_resolver import read_scene_task, normalized_intent_hash, context_hash
            intent, physical, document = read_scene_task(package)
            if intent.get('safety', {}).get('execution_backend', 'fake') != args.backend:
                raise RuntimeError('authored execution backend differs from requested backend')
            if args.retreat_distance is not None or args.task_request is not None:
                raise ValueError('Authored TaskIntent cannot be overridden by a task request or retreat argument')
            if (cell.get('normalized_intent_sha256') != normalized_intent_hash(intent) or
                    cell.get('task_intent_resolution', {}).get('context_sha256') != context_hash(intent, physical, document)):
                raise ValueError('TASK_HANDOFF_STALE: Generate the saved task before planning')
            selection = intent['pick']['selection']
            if selection['source_type'] == 'manual_simulated' and not args.replay and args.backend != 'simulator':
                raise ValueError('Manual simulated source requires explicit replay input')
            filters = selection['object_filter']
            task = dict(action='pick_and_place', selection_policy='task_semantics',
                        target_class=filters.get('class_id'), source_zone=selection['zone_ref'],
                        destination_zone=intent['place']['target']['region_ref'],
                        max_age_seconds=filters['max_age_seconds'],
                        min_confidence=filters.get('min_confidence') or 0.,
                        allow_missing_confidence=filters.get('min_confidence') is None)
        else:
            if not args.task_request:
                raise ValueError('Legacy task request missing; Save and Generate a TaskIntent v2 task')
            task = inputs.task_request(yaml.safe_load(Path(args.task_request).read_text()), cell)
        stage('ACQUIRE_OBJECTS')
        snapshot = yaml.safe_load(Path(args.detections).read_text())
        if args.replay:
            snapshot = inputs.replay_snapshot(snapshot, time.time())
        objects = inputs.normalize(snapshot, time.time(), _PLANNER)
        stage('FILTER_TARGETS')
        if authored:
            from task_intent_resolver import select_observations, scene_resolution
            eligible = select_observations(intent, physical, objects, time.time())
            rejected = {o['id']: 'outside_authored_selection' for o in objects if o not in eligible}
            expected_resolution = None if args.resolve_task else scene_resolution(
                package, intent, physical, document, require_ready=True)
            if expected_resolution is not None:
                from task_intent_resolver import consume_resolution
                consume_resolution(expected_resolution, intent, physical, document, generated_cell=cell)
        else:
            eligible, rejected = inputs.filter_targets(objects, task, cell, time.time(), _PLANNER)
        summary.update(task_request=task, normalized_objects=objects, rejected_objects=rejected)
        params = call(params_client, GetParameters.Request(names=['use_fake_hardware','allow_trajectory_execution','robot_description'])).values
        from hashlib import sha256
        contract['robot_model_sha256'] = sha256(params[2].string_value.encode()).hexdigest()
        summary['trajectory_execution_enabled'] = params[1].bool_value
        components = call(hardware_client,ListHardwareComponents.Request()).component
        if args.backend == 'fake':
            if args.simulator_receipt is not None:
                raise RuntimeError('simulator receipt contradicts fake backend')
            summary['fake_hardware_guard'] = fake_hardware_evidence(params, components)
        else:
            if args.simulator_receipt is None:
                raise RuntimeError('simulator backend requires a live receipt')
            from simulator_backend import live_identity
            settings = call(params_client, GetParameters.Request(names=['execution_backend', 'use_sim_time'])).values
            if settings[0].string_value != 'simulator' or params[0].bool_value:
                raise RuntimeError('simulator backend contradicts MoveIt launch configuration')
            controllers = call(node.create_client(ListControllers, '/controller_manager/list_controllers'), ListControllers.Request()).controller
            interfaces = call(node.create_client(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces'), ListHardwareInterfaces.Request()).command_interfaces
            summary['backend_identity'] = live_identity(args.simulator_receipt, params[2].string_value,
                components, controllers, interfaces, node.get_node_names_and_namespaces(), settings[1].bool_value)
            # Physical position control needs reserve beyond a tangent first
            # contact. This opt-in is confined to the verified simulator path.
            contract['simulator_closure_reserve'] = True
            from simulator_observations import verify_snapshot_binding
            verify_snapshot_binding(snapshot, summary['backend_identity']['receipt_sha256'])
        if args.start and not params[1].bool_value:
            raise RuntimeError('fake execution disabled; launch allow_trajectory_execution:=true')
        support_adapters = call(params_client, GetParameters.Request(names=['ompl.request_adapters'])).values[0].string_value
        mimics = [(j.attrib['name'], m.attrib['joint'], float(m.get('multiplier',1)), float(m.get('offset',0)))
            for j in ET.fromstring(params[2].string_value).findall('joint') for m in j.findall('mimic')]
        stage('UPDATE_PLANNING_SCENE')
        home = dict(zip(contract['home_joint_names'],contract['home_joint_positions']))
        initial = wait_for_robot_baseline(scene_now, home, mimics, min(10.0, args.timeout))
        if initial.robot_state.attached_collision_objects:
            raise RuntimeError('existing attachment requires explicit recovery')
        existing_ids = {o.id for o in initial.world.collision_objects}
        additions=observations_to_insert(objects,{o.id:collision_object_dict(o) for o in initial.world.collision_objects},args.backend)
        manifest = yaml.safe_load((package/'config/moveit_collision_objects.yaml').read_text())
        if not {o['id'] for o in manifest['objects']}.issubset(existing_ids):
            raise RuntimeError('generated environment collisions missing')
        if additions:apply(inputs.scene_diff(additions))
        initial = scene_now()
        initial.robot_state.is_diff = False
        initial.is_diff = True
        from rosidl_runtime_py.convert import message_to_ordereddict
        def evidence_scene(name, scene):
            path = Path(args.summary_output).parent / (name + '.json')
            path.write_text(json.dumps(message_to_ordereddict(scene), indent=2)+'\n')
        evidence_scene('planning_scene_before', initial)
        summary['inserted_object_ids'] = [o['id'] for o in objects]
        actual = {o.id:collision_object_dict(o) for o in initial.world.collision_objects}
        for obj in objects:
            if not observation_geometry_matches(obj, actual.get(obj['id'])):
                raise RuntimeError('normalized scene insertion mismatch')
        # A non-home start is not silently corrected with motion before validation.
        assert_joint_match(initial.robot_state, updated_state(initial.robot_state, dict(home, gripper_finger1_joint=0.0), mimics),0.001)
        baseline = copy.deepcopy(initial.allowed_collision_matrix)
        destination = load_canonical_place_target(package, task['destination_zone'])
        retreat = args.retreat_distance if args.retreat_distance is not None else contract['retreat_distance_m']
        if not math.isfinite(retreat) or retreat <= 0:
            raise RuntimeError('retreat distance must be finite and positive')
        deadline = time.monotonic()+args.timeout
        from full_cycle_preplanner import PreplanOperations
        def extraction_candidates(view, observation, candidate, lift_distance):
            from pile_extraction import extraction_intents
            neighbors = [item for obj in view.world.collision_objects
                         if obj.id != observation['id'] and (item := collision_object_dict(obj)) is not None]
            return extraction_intents(observation, neighbors, candidate.candidate_id, lift_distance)
        operations = PreplanOperations(plan_segment=plan_segment, fk=fk,
            state_validity=lambda state: call(validity_client,
                GetStateValidity.Request(robot_state=state, group_name='')),
            updated_state=lambda state, positions: updated_state(state, positions, mimics),
            pose_message=pose_message, translated_pose=translated_pose,
            target_contact_matrix=target_contact_matrix, verify_selected_contacts=verify_selected_contacts,
            private_attachment=private_attachment, object_pose_after_motion=object_pose_after_motion,
            place_detachment_diff=place_detachment_diff, stage=stage,
            extraction_candidates=extraction_candidates if authored else None)
        if authored:
            cycle = plan_authored_cycle(initial_scene=initial, intent=intent, environment=physical,
                cell=document, targets=eligible, contract=contract, operations=operations,
                deadline=deadline, summary=summary, resolved=expected_resolution,
                planning_time=args.segment_planning_time)
            destination = summary['task_intent_resolution']['place_resolution']['destination']
            if expected_resolution is not None and expected_resolution['resolution_sha256'] != summary['resolution_sha256']:
                raise ValueError('TASK_RESOLUTION_DIVERGED: current planning differs from generated resolution; resolve and regenerate')
        else:
            cycle = plan_legacy_cycle(initial_scene=initial, targets=eligible, destination=destination,
                contract=dict(contract, max_age_seconds=task['max_age_seconds'], retreat_distance_m=retreat),
                operations=operations, deadline=deadline, summary=summary)
        selected_id = cycle['object_id']
        summary.update(selected_object_id=selected_id,selected_grasp_index=cycle['grasp_index'],
                       selected_extraction_intent=copy.deepcopy(cycle.get('extraction_intent')),full_cycle_prevalidated=True,
                       full_cycle_plan_success=True,plan_metadata=[s['metadata'] for s in cycle['steps'] if s['kind']=='motion'])
        stage('VERIFY_PREPLAN_UNCHANGED')
        assert_scene_match(scene_now(),initial)
        summary['prevalidation_left_live_scene_unchanged'] = True
        if authored:
            from task_intent_resolver import consume_resolution, read_scene_task, write_resolution_artifacts
            current_intent, current_physical, current_document = read_scene_task(package)
            consume_resolution(summary['task_intent_resolution'], current_intent, current_physical, current_document)
            if args.resolve_task:
                write_resolution_artifacts(summary['task_intent_resolution'], package / 'generated')
        if not args.start:
            summary['result'] = 'PLAN_ONLY'
            return 0
        if args.simulator_commission and args.backend != 'simulator':
            raise RuntimeError('commissioning is simulator-only')
        if args.backend == 'simulator':
            if not args.simulator_commission:
                raise RuntimeError('SIMULATOR_EXECUTION_UNCOMMISSIONED: explicit commissioning trial required')
            # Planning can take minutes. Recheck the complete positive identity
            # at the motion boundary before starting the telemetry consumer.
            current_params=call(params_client,GetParameters.Request(names=[
                'execution_backend','use_fake_hardware','allow_trajectory_execution','use_sim_time','robot_description',
                'simulator_commissioning','capabilities','disable_capabilities'])).values
            if (current_params[0].string_value!='simulator' or current_params[1].bool_value or
                    not current_params[2].bool_value or current_params[4].string_value!=params[2].string_value):
                raise RuntimeError('simulator configuration changed before motion')
            motion_controllers=call(node.create_client(ListControllers,'/controller_manager/list_controllers'),ListControllers.Request()).controller
            summary['motion_backend_identity']=live_identity(args.simulator_receipt,current_params[4].string_value,
                call(hardware_client,ListHardwareComponents.Request()).component,
                motion_controllers,
                call(node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces'),ListHardwareInterfaces.Request()).command_interfaces,
                node.get_node_names_and_namespaces(),current_params[3].bool_value)
            from simulator_backend import commissioning_capability_identity
            summary['commissioning_capability']=commissioning_capability_identity(node,args.simulator_receipt,
                current_params[5].bool_value,current_params[6].string_value,current_params[7].string_value)
            verify_snapshot_binding(snapshot,summary['motion_backend_identity']['receipt_sha256'])
            from simulator_execution import Measurements, ContactGuard, measured_attachment, require_trial_evidence
            if args.simulator_commission=='full-cycle':
                summary['commissioning_prerequisites']=require_trial_evidence(
                    args.commission_evidence,summary['commissioning_capability'])
            measurements=Measurements(node,args.simulator_receipt,Path(args.summary_output).with_suffix('.measurements.jsonl'))
            until=time.monotonic()+5
            while measurements.latest is None and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            support=next((s['metadata']['initial_support_contact'] for s in cycle['steps'] if s.get('metadata',{}).get('initial_support_contact')),None)
            selected=next(o for o in objects if o['id']==selected_id)
            contact_guard=ContactGuard(measurements,selected_id,selected['dimensions'],contract['allowed_touch_links'],support,baseline,contract['tool_link'])
            contact_guard.bind_pile(objects,dict(resolution_sha256=summary['resolution_sha256'],
                execution_attempt=measurements.receipt['run_id']+':'+str(time.monotonic_ns()),
                selected_grasp_index=summary['selected_grasp_index'],
                commissioning_sha256=summary['commissioning_capability']['sha256']))
            contact_guard.arm_names=set(contract['home_joint_names'])
            initial_joints=measurements.joints(measurements.fresh())
            leaders=set(initial_joints)-contact_guard.arm_names-{j.get('name') for j in measurements.robot.findall('joint') if j.find('mimic') is not None}
            if len(leaders)!=1:raise RuntimeError('ambiguous measured gripper leader')
            contact_guard.open_position=initial_joints[next(iter(leaders))][0]
            summary['measured_initial_object_poses']={}
            for obj in objects:
                actual=measurements.object_pose(measurements.fresh(),obj['object_id'])
                summary['measured_initial_object_poses'][obj['id']]=actual
                if math.dist(actual[:3],obj['pose'][:3])>.001:
                    raise RuntimeError('measured object moved since authoritative resolution')
            contact_guard.check(measurements.fresh());measurements.arm()
            contact_guard.validate_current=measured_fcl
            execution_monitor=contact_guard.drain
            if args.simulator_commission=='cancel':
                from simulator_execution import ControllerCancellationAudit
                controller_audit=ControllerCancellationAudit(node,motion_controllers,contract['home_joint_names'])
                until=time.monotonic()+.2
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        if args.backend == 'fake' and any(s.get('metadata', {}).get('initial_support_contact') for s in cycle['steps']):
            raise RuntimeError('SUPPORT_CONTACT_PLAN_ONLY: execution of initial support separation is not commissioned')
        fresh = select_observations(intent, physical, objects, time.time()) if authored else inputs.filter_targets(objects,task,cell,time.time(),_PLANNER)[0]
        if selected_id not in {o['id'] for o in fresh}:
            raise RuntimeError('selected observation expired before execution')
        expected = initial
        for step in cycle['steps']:
            if args.simulator_commission in ('cancel','telemetry') and step['stage']!='PREPLAN_APPROACH':
                raise RuntimeError(f'{args.simulator_commission} trial cannot advance beyond the approach')
            label = step['stage'].replace('PREPLAN_','EXECUTE_')
            stage(label)
            if measurements:
                # The full-cycle authority supplied every approach, grasp and lift.
                # This branch replaces mock exact-state bookkeeping with live evidence.
                contact_guard.phase={'PREPLAN_APPROACH':'approach','PREPLAN_GRASP':'descent',
                    'PREPLAN_CLOSE_GRIPPER':'closing','PREPLAN_LIFT':'lift','PREPLAN_TRANSFER':'transfer',
                    'PREPLAN_PLACE':'place','PREPLAN_OPEN_GRIPPER':'opening','PREPLAN_RETREAT':'released',
                    'PREPLAN_HOME':'released'}.get(step['stage'],contact_guard.phase)
                if step['kind']=='motion':
                    if not contact_guard.held:
                        apply(PlanningScene(is_diff=True,allowed_collision_matrix=step['before'].allowed_collision_matrix))
                    contact_guard.drain()
                    if step['stage']=='PREPLAN_LIFT':contact_guard.begin_pile_separation(measurements.fresh())
                    controlled_cancel=args.simulator_commission=='cancel' and step['stage']=='PREPLAN_APPROACH'
                    summary['execution_attempted']=True
                    result=action(execute_client,ExecuteTrajectory.Goal(trajectory=step['trajectory']),120)
                    if step['stage']=='PREPLAN_CLOSE_GRIPPER':summary['close_terminal_wall_ns']=time.time_ns()
                    controlled_cancel=False
                    contact_guard.drain()
                    summary.setdefault('execution_results',[]).append(dict(stage=label,code=result.error_code.val,action_status=4))
                    if step['stage']=='PREPLAN_APPROACH' and args.simulator_commission=='telemetry':
                        summary['motion_stop_verified']=wait_stopped(monitor_contacts=True)
                        if not summary['motion_stop_verified']:raise RuntimeError('motion telemetry trial did not reach measured stationary state')
                        summary['motion_telemetry']=measurements.metrics()
                        measured_reconcile()
                        summary.update(result='MOTION_TELEMETRY_PASS',commission_trial='telemetry',
                                       full_cycle_execution_success=False)
                        break
                    if step['stage']=='PREPLAN_LIFT' and args.simulator_commission in ('contact-release','full-cycle'):
                        monitored_hold(1.)
                        summary['lift_hold_measurement']=measurements.fresh()
                        pile=contact_guard.pile_certificate
                        if support is None and not pile['certified_set']:raise RuntimeError('physical lift has no certified initial support')
                        floor=support['floor_z'] if support else pile['initial_bottom_m']
                        summary['verified_lift_clearance_m']=contact_guard.bottom(measurements.object_pose(measurements.fresh(),contact_guard.name))-floor
                        if ((support and (not contact_guard.separation or not contact_guard.separation.expired)) or
                            not set(pile['certified_set']).issubset(contact_guard.pile_expired) or summary['verified_lift_clearance_m']<.01):
                            raise RuntimeError('physical lift separation not verified')
                        if args.simulator_commission=='contact-release':
                            # Plan release in the actual lifted scene with the existing planner.
                            deadline=time.monotonic()+30
                            release=plan_segment(scene_now(),'COMMISSION_RELEASE',{contact_guard.leader:contact_guard.open_position},group='gripper')
                            contact_guard.phase='opening'
                            action(execute_client,ExecuteTrajectory.Goal(trajectory=release['trajectory']),30)
                            contact_guard.phase='released';contact_guard.held=None
                            measured_reconcile()
                            monitored_hold(2.)
                            from simulator_execution import verify_release
                            summary['release_evidence']=verify_release(contact_guard,summary['lift_hold_measurement'])
                            measured_reconcile()
                            summary.update(result='CONTACT_RELEASE_PASS',full_cycle_execution_success=False)
                            break
                    if step['stage']=='PREPLAN_PLACE' and args.simulator_commission=='full-cycle':
                        summary['pre_release_measurement']=measurements.fresh()
                elif step['kind']=='attach':
                    contact_guard.pile_binding.update(close_goal_uuid=summary['owned_execution_goal']['uuid'],
                        close_goal_accepted_wall_ns=summary['owned_execution_goal']['wall_ns'],
                        close_goal_terminal_wall_ns=summary['close_terminal_wall_ns'])
                    # Observe exact post-close contacts while the existing stop
                    # window settles. Freeze this continuously checked set before
                    # retention admission or any further commanded arm motion.
                    contact_guard.begin_pile_admission()
                    summary['motion_stop_verified']=wait_stopped(monitor_contacts=True)
                    if not summary['motion_stop_verified']:
                        raise RuntimeError('physical closure did not converge')
                    # Every current grasp must prove physical retention before
                    # planning attachment or further arm motion. Keep the frozen
                    # contact certificate and held reference throughout the hold.
                    summary['measured_object_in_tool']=contact_guard.establish()
                    summary['stationary_hold_start']=contact_guard.checked_current()
                    monitored_hold(1.1)
                    summary['stationary_retention']=contact_guard.retention_evidence()
                    retained_sample=contact_guard.checked_current()
                    summary['stationary_hold_end']=retained_sample
                    if args.simulator_commission=='stationary':
                        # This gate proves physical retention only. No planning-scene
                        # attachment or later arm motion is used as grasp evidence.
                        # Drop only the executor's held-state bookkeeping after
                        # evidence capture; the physics object was never parented.
                        contact_guard.held=None;contact_guard.separation=None
                        measured_reconcile()
                        summary.update(result='STATIONARY_RETENTION_PASS',commission_trial='stationary',
                                       full_cycle_execution_success=False)
                        break
                    summary['closure_measurement']=retained_sample
                    apply(measured_attachment(step['original'],contract,measurements,retained_sample))
                    contact_guard.planning_attached=True
                    apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                else:
                    contact_guard.phase='released';monitored_hold(2.)
                    if args.simulator_commission=='full-cycle':
                        from simulator_execution import verify_settled_release
                        held_sample=summary.get('pre_release_measurement')
                        if held_sample is None:raise RuntimeError('full-cycle release lacks a measured pre-release state')
                        summary['release_evidence']=verify_settled_release(contact_guard,held_sample)
                        contact_guard.held=None;contact_guard.separation=None
                    measured_reconcile()
                summary['stages'].append(label)
                continue
            assert_scene_match(scene_now(),expected,selected_id)
            matrix = step['before'].allowed_collision_matrix
            if collision_matrix_signature(matrix) != collision_matrix_signature(expected.allowed_collision_matrix):
                apply(PlanningScene(is_diff=True,allowed_collision_matrix=matrix))
            assert_scene_match(scene_now(),step['before'],selected_id)
            if step['kind']=='motion':
                require_prevalidated_execution(args.start, cycle)
                summary['execution_attempted'] = True
                result = action(execute_client,ExecuteTrajectory.Goal(trajectory=step['trajectory']),60)
                summary.setdefault('execution_results',[]).append(dict(stage=label,code=result.error_code.val,action_status=4))
            elif step['kind']=='attach':
                apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                measured = call(validity_client,GetStateValidity.Request(robot_state=scene_now().robot_state,group_name=''))
                verify_selected_contacts(measured.contacts,selected_id,contract['allowed_touch_links'])
                apply(attachment_diff(step['original'],contract['grasp_frame'],contract['allowed_touch_links'],False))
            else:
                actual_tool = fk(scene_now().robot_state,contract['tool_link'])
                achieved = object_pose_after_motion(step['original'],step['tool_at_grasp'].pose,actual_tool.pose)
                apply(place_detachment_diff(step['original'],contract['grasp_frame'],achieved[:3],achieved[3:]))
                summary['achieved_place_pose'] = achieved
            expected = step['after']
            # Restore baseline immediately after attachment, or stage contact
            # allowances only when the next planned segment requires them.
            if collision_matrix_signature(matrix) != collision_matrix_signature(expected.allowed_collision_matrix):
                apply(PlanningScene(is_diff=True,allowed_collision_matrix=expected.allowed_collision_matrix))
            measured_scene = scene_now()
            assert_scene_match(measured_scene,expected,selected_id)
            if step['kind'] == 'attach':
                evidence_scene('planning_scene_attached', measured_scene)
                summary['attachment_state'] = attachment_status(measured_scene, selected_id, contract['grasp_frame'])
                summary['attach_verified'] = summary['attachment_state']['valid']
            elif step['kind'] == 'detach':
                evidence_scene('planning_scene_after', measured_scene)
                summary['detach_verified'] = not measured_scene.robot_state.attached_collision_objects and any(
                    o.id == selected_id for o in measured_scene.world.collision_objects)
            summary['stages'].append(label)
        if measurements:
            measured_reconcile()
            if summary['result'] in ('MOTION_TELEMETRY_PASS','STATIONARY_RETENTION_PASS','CONTACT_RELEASE_PASS'):return 0
            if args.simulator_commission!='full-cycle':
                raise RuntimeError('bounded simulator trial ended without an accepted result')
            # Final measured acceptance: physics state, planning state, robot home,
            # destination containment and release separation must all agree.
            final_sample=measurements.fresh();final_joints=measurements.joints(final_sample)
            for name,value in home.items():
                if name not in final_joints or abs(final_joints[name][0]-value)>.001:
                    raise RuntimeError(f'full-cycle measured home mismatch: {name}')
            if abs(final_joints[contact_guard.leader][0]-contact_guard.open_position)>.01:
                raise RuntimeError('full-cycle measured gripper is not open')
            final_pose=measurements.object_pose(final_sample,contact_guard.name)
            from physical_destination import check_object_containment
            check_object_containment(destination,final_pose,selected['dimensions'],clearance=contract.get('placement_clearance_m',.001))
            pre=summary.get('pre_release_measurement')
            before_relative=compose_pose(inverse_pose(measurements.frame(pre,contract['tool_link'])),
                                         measurements.object_pose(pre,contact_guard.name))
            after_relative=compose_pose(inverse_pose(measurements.frame(final_sample,contract['tool_link'])),final_pose)
            relative_separation=math.dist(before_relative[:3],after_relative[:3])
            if relative_separation<.01:raise RuntimeError('full-cycle release did not separate object from retreating tool')
            final_scene=scene_now()
            if final_scene.robot_state.attached_collision_objects:
                raise RuntimeError('full-cycle final planning scene retains an attachment')
            if collision_matrix_signature(final_scene.allowed_collision_matrix)!=collision_matrix_signature(baseline):
                raise RuntimeError('full-cycle final ACM differs from baseline')
            final_valid=call(validity_client,GetStateValidity.Request(robot_state=final_scene.robot_state,group_name=''))
            if not final_valid.valid:raise RuntimeError('full-cycle final robot state is in collision')
            summary.update(result='PASS',full_cycle_execution_success=True,
                full_cycle_physical_acceptance=dict(final_pose=final_pose,
                    release_relative_separation_m=relative_separation,
                    final_sim_ns=final_sample['sim_ns'],final_collision_valid=True,
                    baseline_acm_restored=True,attached_ids=[]))
            return 0
        final = scene_now()
        assert_scene_match(final, expected, selected_id)
        intended_home = dict(home, gripper_finger1_joint=0.0)
        assert_joint_match(final.robot_state, updated_state(final.robot_state, intended_home, mimics), 0.001)
        summary['final_robot_state'] = dict(
            actual_joints=dict(zip(final.robot_state.joint_state.name, final.robot_state.joint_state.position)),
            intended_home=intended_home, tolerance_rad=0.001)
        summary['final_collision_valid'] = call(validity_client,
            GetStateValidity.Request(robot_state=final.robot_state, group_name='')).valid
        if not summary['final_collision_valid']:
            raise RuntimeError('final home state is in collision')
        summary['baseline_acm_restored'] = collision_matrix_signature(final.allowed_collision_matrix) == collision_matrix_signature(baseline)
        placed = [o for o in final.world.collision_objects if o.id == selected_id]
        if len(placed) != 1 or final.robot_state.attached_collision_objects:
            raise RuntimeError('final selected object is not uniquely detached in world')
        summary['final_object_state'] = collision_object_dict(placed[0])
        from physical_destination import check_object_containment
        check_object_containment(destination, summary['final_object_state']['pose'],
                                 summary['final_object_state']['dimensions'], clearance=0.001)
        summary['physical_destination_verified'] = True
        summary['destination'] = destination
        summary['placement_error_m'] = math.dist(summary['final_object_state']['pose'][:3], destination['pose_xyz'])
        summary['final_planning_scene'] = dict(world_ids=[o.id for o in final.world.collision_objects],
            attached_ids=[o.object.id for o in final.robot_state.attached_collision_objects],distractors_unchanged=True)
        evidence_scene('planning_scene_final', final)
        summary['home_verified'] = True  # Final assert_scene_match includes planned home joints.
        summary['destination_verified'] = math.dist(summary['achieved_place_pose'][:3], destination['pose_xyz']) <= 0.003
        if not summary['destination_verified'] or summary['placement_error_m'] > 0.003 or not summary['baseline_acm_restored']:
            raise RuntimeError('final destination or baseline ACM verification failed')
        stage('COMPLETE')
        summary.update(result='PASS',full_cycle_execution_success=True)
    except (Exception,KeyboardInterrupt) as exc:
        summary.update(failed_stage=exc.stage if isinstance(exc,CandidateFailure) else summary['current_stage'],failure=str(exc))
        stage('FAILED')
        summary['recovery_required'] = summary['execution_attempted']
        resolution = summary.get('task_intent_resolution')
        if args.resolve_task and resolution is not None and resolution['readiness_status'] == 'BLOCKED':
            try:
                from task_intent_resolver import consume_resolution, read_scene_task, write_resolution_artifacts
                current_intent, current_physical, current_document = read_scene_task(package)
                consume_resolution(resolution, current_intent, current_physical, current_document, require_ready=False)
                write_resolution_artifacts(resolution, package / 'generated')
            except (ValueError, OSError) as evidence_error:
                summary['resolution_write_blocker'] = str(evidence_error)
        # Do not erase held/placed objects or command a recovery trajectory.
    finally:
        if summary['execution_attempted'] and summary['result'] != 'PASS' and not rclpy.ok():
            summary['recovery_inspection_skipped'] = 'ROS context already invalid'
        if summary['execution_attempted'] and summary['result'] != 'PASS' and rclpy.ok():
            try:
                if measurements:
                    apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                    if summary.get('motion_stop_verified') or wait_stopped():measured_reconcile()
                    else:raise RuntimeError('cannot reconcile before measured motion stop')
                else:apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                recovery = scene_now()
                summary['recovery_scene'] = dict(
                    attached_ids=[o.object.id for o in recovery.robot_state.attached_collision_objects],
                    world_ids=[o.id for o in recovery.world.collision_objects],
                    contact_acm_restored=collision_matrix_signature(recovery.allowed_collision_matrix)==collision_matrix_signature(baseline))
            except Exception as recovery_error:
                summary['recovery_inspection_failure'] = str(recovery_error)
        if args.simulator_commission=='cancel' and summary.get('failure')=='CONTROLLED_CANCELLATION':
            try:
                from simulator_execution import require_cancellation_acceptance
                require_cancellation_acceptance(summary)
                summary['result']='CANCELLATION_TRIAL_PASS'
            except RuntimeError as acceptance_error:
                summary['cancellation_acceptance_failure']=str(acceptance_error)
        if contact_guard and contact_guard.pile_certificate is not None:
            summary['pile_contact_certification']=contact_guard.pile_certificate
        if measurements:measurements.close()
        node.destroy_node()
        rclpy.try_shutdown()
        summary['shutdown_clean'] = not rclpy.ok()
        Path(args.summary_output).write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
        print(json.dumps(summary,indent=2,sort_keys=True))
    return 0 if summary['result'] in ('PASS','CANCELLATION_TRIAL_PASS','MOTION_TELEMETRY_PASS','STATIONARY_RETENTION_PASS','CONTACT_RELEASE_PASS') else 1


if __name__ == '__main__':
    raise SystemExit(main())
