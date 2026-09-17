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
                        contract, operations, deadline, summary, resolved=None):
    """The resolver selects; the existing preplanner alone proves feasibility."""
    from task_intent_resolver import resolve_task_intent
    from full_cycle_preplanner import preplan_full_cycle
    cycles = {}

    def evaluate(request):
        effective = dict(contract)
        grasp = request['intent']['pick']['grasp']
        place = request['intent']['place']['placement']
        effective.update(max_age_seconds=intent['pick']['selection']['object_filter']['max_age_seconds'],
                         retreat_distance_m=grasp['lift']['distance_m'],
                         place_approach_distance_m=place['approach']['distance_m'],
                         place_retreat_distance_m=place['retreat']['distance_m'],
                         placement_clearance_m=place['clearance_m'],
                         task_intent=request['intent'])
        result = preplan_full_cycle(initial_scene=initial_scene,
            observation=request['observation'], candidate=request['candidate'],
            destination=request['destination'], contract=effective,
            operations=operations, deadline=deadline)
        summary['candidate_attempts'].append({
            'candidate_id': result.candidate_id, 'object_id': request['observation']['id'],
            'stages': result.stages, 'checks': result.checks, 'reason_code': result.reason_code})
        if result.success:
            cycles[result.candidate_id, request['observation']['id']] = result.cycle
        return {'success': result.success, 'checks': result.checks,
                'reason_code': result.reason_code, 'reason': result.reason}

    resolution = resolve_task_intent(intent, environment, cell, targets, evaluate, now=time.time(), resolved=resolved)
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
    parser.add_argument('--scene-package', default='ur5_2f_test')
    parser.add_argument('--task-request')
    parser.add_argument('--resolve-task', action='store_true', help='Resolve the saved authored task and write derived evidence after plan-only verification')
    parser.add_argument('--detections', required=True)
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--start', action='store_true')
    parser.add_argument('--timeout', type=float, default=180.0, help='Total candidate search budget in seconds')
    parser.add_argument('--retreat-distance', type=float)
    args = parser.parse_args()
    import yaml
    import xml.etree.ElementTree as ET
    import rclpy
    from rclpy.action import ActionClient
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents, Constraints,
        JointConstraint, MotionPlanRequest)
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListHardwareComponents
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
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt action unavailable')
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
        try:
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError('action result timed out')
        except BaseException:
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
                except Exception as cancel_error:
                    summary['cancellation_failure'] = str(cancel_error)
            else:
                summary['cancellation_failure'] = 'ROS context already invalid'
            raise
        response = future.result()
        if response.status != 4 or response.result.error_code.val != 1:
            raise MoveItActionFailure(response.status, response.result.error_code.val)
        return response.result
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
    def plan_segment(view, name, goal, group=None, straight=False, initial_support=None):
        stage(name)
        if time.monotonic() > deadline:
            raise RuntimeError('candidate search budget exhausted')
        before = copy.deepcopy(view)
        request = MotionPlanRequest(group_name=group or contract['planning_group'],
            start_state=copy.deepcopy(view.robot_state), num_planning_attempts=1,
            allowed_planning_time=3.0, max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        request.start_state.is_diff = False
        if isinstance(goal, dict):
            request.goal_constraints = [joint_constraints(goal)]
            goal_state = updated_state(view.robot_state, goal, mimics)
        if straight:
            # Chain short collision-planned moves in the same private scene.
            # Validate FK along each returned trajectory, rather than treating
            # endpoint feasibility as proof of a straight collision-safe path.
            start_pose = fk(view.robot_state, contract['tool_link'])
            a, b = pose_values(start_pose.pose), pose_values(goal.pose)
            count = max(1, math.ceil(math.dist(a[:3], b[:3]) / 0.005))
            combined = None
            elapsed_ns = 0
            total_planning_time = 0.0
            support = None
            if name == 'PREPLAN_LIFT' and len(view.robot_state.attached_collision_objects) == 1:
                object_id = view.robot_state.attached_collision_objects[0].object.id
                original = next(o for o in initial.world.collision_objects if o.id == object_id)
                eligible_supports = legitimate_support_ids(collision_object_dict(original), cell['environment'], manifest)
                measured = call(validity_client, GetStateValidity.Request(robot_state=view.robot_state, group_name=''))
                contacts = contacts_in_planned_scene(measured.contacts, view, initial)
                floor_contacts = []
                for c in contacts:
                    if c.contact_body_1 == object_id and c.body_type_1 == 2 and c.contact_body_2 in eligible_supports and c.normal.z < -.999999:
                        floor_contacts.append((c.contact_body_2, c.position.z))
                    elif c.contact_body_2 == object_id and c.body_type_2 == 2 and c.contact_body_1 in eligible_supports and c.normal.z > .999999:
                        floor_contacts.append((c.contact_body_1, c.position.z))
                if floor_contacts and len({p[0] for p in floor_contacts}) == 1:
                    if 'workcell/InitialSupportContact' not in support_adapters.split():
                        raise RuntimeError('SUPPORT_CONTACT_ADAPTER_MISSING: regenerate and launch the current MoveIt configuration')
                    support = dict(object_id=object_id, support_id=floor_contacts[0][0],
                                   floor_z=floor_contacts[0][1], tool_link=contract['tool_link'])
            for i in range(1, count+1):
                waypoint = pose_message([x+(y-x)*i/count for x,y in zip(a[:3],b[:3])] + b[3:])
                part = plan_segment(view, name, waypoint, group, initial_support=support if i == 1 else None)
                trajectory = part['trajectory']
                for point in trajectory.joint_trajectory.points:
                    sample = updated_state(view.robot_state, dict(zip(trajectory.joint_trajectory.joint_names, point.positions)), mimics)
                    actual_pose = pose_values(fk(sample, contract['tool_link']).pose)
                    if not pose_within_cartesian_corridor(actual_pose, a, b):
                        from full_cycle_preplanner import MotionFeasibilityFailure
                        failure = MotionFeasibilityFailure('planned contact/retreat path leaves the Cartesian corridor')
                        failure.details.update(path_pose=actual_pose, corridor_start=a, corridor_goal=b)
                        raise failure
                if combined is None:
                    combined = copy.deepcopy(trajectory)
                else:
                    if combined.joint_trajectory.joint_names != trajectory.joint_trajectory.joint_names:
                        raise RuntimeError('waypoint trajectory joint order changed')
                    for point in trajectory.joint_trajectory.points[1:]:
                        point = copy.deepcopy(point)
                        stamp = point.time_from_start.sec*1000000000 + point.time_from_start.nanosec + elapsed_ns
                        point.time_from_start.sec, point.time_from_start.nanosec = divmod(stamp,1000000000)
                        combined.joint_trajectory.points.append(point)
                last = combined.joint_trajectory.points[-1].time_from_start
                elapsed_ns = last.sec*1000000000+last.nanosec
                total_planning_time += part['metadata']['planning_time']
                view = copy.deepcopy(part['after'])
            return dict(kind='motion',stage=name,before=before,after=view,trajectory=combined,
                metadata=dict(stage=name,success=True,moveit_code=1,planning_time=total_planning_time,
                    points=len(combined.joint_trajectory.points),cartesian_waypoints=count,
                    initial_support_contact=support,
                    attached_ids=[o.object.id for o in before.robot_state.attached_collision_objects],
                    world_ids=[o.id for o in before.world.collision_objects]))
        if not isinstance(goal, dict):
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = contract['planning_group']
            ik_request.ik_request.ik_link_name = contract['tool_link']
            ik_request.ik_request.pose_stamped = goal
            ik_request.ik_request.robot_state = view.robot_state
            # Avoid rejecting a pose solely because IK selected a colliding arm
            # branch. The live service is usable only while its collision scene
            # matches this private view; otherwise preserve seed continuity and
            # let the private MoveGroup plan establish feasibility. Neither IK
            # outcome replaces the complete candidate motion checks.
            ik_request.ik_request.avoid_collisions = ik_scene_matches_live(view, initial)
            ik_request.ik_request.timeout.sec = 1
            ik = call(ik_client, ik_request)
            if ik.error_code.val != 1:
                from full_cycle_preplanner import MotionFeasibilityFailure
                raise MotionFeasibilityFailure(f'IK failed: {ik.error_code.val}', moveit_code=ik.error_code.val)
            values = dict(zip(ik.solution.joint_state.name,ik.solution.joint_state.position))
            request.goal_constraints = [joint_constraints({n:values[n] for n in contract['home_joint_names']})]
            goal_state = updated_state(view.robot_state,
                                       {n: values[n] for n in contract['home_joint_names']}, mimics)
        if initial_support is not None:
            request.path_constraints.name = 'workcell_initial_support_contact:' + json.dumps(initial_support, sort_keys=True)
        goal_msg = MoveGroup.Goal(request=request)
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.planning_scene_diff = copy.deepcopy(view)
        # OMPL can return an invalid sampled path for a feasible fixed goal.
        # Retry that identical plan-only request, never a different candidate,
        # target, policy or execution action. All collision checks remain active.
        for attempt in range(3):
            try:
                result = action(plan_client, goal_msg, 12)
                break
            except MoveItActionFailure as exc:
                if exc.code != -2 or attempt == 2 or time.monotonic() >= deadline:
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
                    {'stage': name, 'moveit_code': exc.code, 'attempt': attempt + 1})
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
        intent = None
        if authored:
            from task_intent_resolver import read_scene_task, normalized_intent_hash, context_hash
            intent, physical, document = read_scene_task(package)
            if args.retreat_distance is not None or args.task_request is not None:
                raise ValueError('Authored TaskIntent cannot be overridden by a task request or retreat argument')
            if (cell.get('normalized_intent_sha256') != normalized_intent_hash(intent) or
                    cell.get('task_intent_resolution', {}).get('context_sha256') != context_hash(intent, physical, document)):
                raise ValueError('TASK_HANDOFF_STALE: Generate the saved task before planning')
            selection = intent['pick']['selection']
            if selection['source_type'] == 'manual_simulated' and not args.replay:
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
        summary['trajectory_execution_enabled'] = params[1].bool_value
        summary['fake_hardware_guard'] = fake_hardware_evidence(params, call(hardware_client,ListHardwareComponents.Request()).component)
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
        if existing_ids.intersection(o['id'] for o in objects):
            raise RuntimeError('runtime IDs already exist; reset the fake scene before replay')
        manifest = yaml.safe_load((package/'config/moveit_collision_objects.yaml').read_text())
        if not {o['id'] for o in manifest['objects']}.issubset(existing_ids):
            raise RuntimeError('generated environment collisions missing')
        apply(inputs.scene_diff(objects))
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
            if actual.get(obj['id']) is None or any(abs(a-b)>1e-7 for a,b in
                    zip(obj['pose']+obj['dimensions'],actual[obj['id']]['pose']+actual[obj['id']]['dimensions'])):
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
        operations = PreplanOperations(plan_segment=plan_segment, fk=fk,
            state_validity=lambda state: call(validity_client,
                GetStateValidity.Request(robot_state=state, group_name='')),
            updated_state=lambda state, positions: updated_state(state, positions, mimics),
            pose_message=pose_message, translated_pose=translated_pose,
            target_contact_matrix=target_contact_matrix, verify_selected_contacts=verify_selected_contacts,
            private_attachment=private_attachment, object_pose_after_motion=object_pose_after_motion,
            place_detachment_diff=place_detachment_diff, stage=stage)
        if authored:
            cycle = plan_authored_cycle(initial_scene=initial, intent=intent, environment=physical,
                cell=document, targets=eligible, contract=contract, operations=operations,
                deadline=deadline, summary=summary, resolved=expected_resolution)
            destination = summary['task_intent_resolution']['place_resolution']['destination']
            if expected_resolution is not None and expected_resolution['resolution_sha256'] != summary['resolution_sha256']:
                raise ValueError('TASK_RESOLUTION_DIVERGED: current planning differs from generated resolution; resolve and regenerate')
        else:
            cycle = plan_legacy_cycle(initial_scene=initial, targets=eligible, destination=destination,
                contract=dict(contract, max_age_seconds=task['max_age_seconds'], retreat_distance_m=retreat),
                operations=operations, deadline=deadline, summary=summary)
        selected_id = cycle['object_id']
        summary.update(selected_object_id=selected_id,selected_grasp_index=cycle['grasp_index'],full_cycle_prevalidated=True,
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
        if any(s.get('metadata', {}).get('initial_support_contact') for s in cycle['steps']):
            raise RuntimeError('SUPPORT_CONTACT_PLAN_ONLY: execution of initial support separation is not commissioned')
        fresh = select_observations(intent, physical, objects, time.time()) if authored else inputs.filter_targets(objects,task,cell,time.time(),_PLANNER)[0]
        if selected_id not in {o['id'] for o in fresh}:
            raise RuntimeError('selected observation expired before execution')
        expected = initial
        for step in cycle['steps']:
            label = step['stage'].replace('PREPLAN_','EXECUTE_')
            stage(label)
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
                apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                recovery = scene_now()
                summary['recovery_scene'] = dict(
                    attached_ids=[o.object.id for o in recovery.robot_state.attached_collision_objects],
                    world_ids=[o.id for o in recovery.world.collision_objects],
                    contact_acm_restored=collision_matrix_signature(recovery.allowed_collision_matrix)==collision_matrix_signature(baseline))
            except Exception as recovery_error:
                summary['recovery_inspection_failure'] = str(recovery_error)
        node.destroy_node()
        rclpy.try_shutdown()
        summary['shutdown_clean'] = not rclpy.ok()
        Path(args.summary_output).write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
        print(json.dumps(summary,indent=2,sort_keys=True))
    return 0 if summary['result']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
