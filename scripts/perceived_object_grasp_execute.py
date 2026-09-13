#!/usr/bin/env python3
"""Plan, execute, attach, and retreat for one perceived object on fake hardware."""

import argparse
import copy
import importlib.util
import json
import math
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
    if not cell_path.exists():
        # Compatibility for older offline fixtures. The generated cell
        # handoff remains authoritative whenever it exists.
        layout_path = package / "layout" / "workcell_studio_layout.yaml"
        if not layout_path.exists():
            raise RuntimeError(f"generated cell handoff is missing: {cell_path}")
        layout = yaml.safe_load(layout_path.read_text(encoding="utf-8")) or {}
        matches = [item for item in layout.get("items", [])
                   if item.get("id") == "target_bin_default"]
        if len(matches) != 1:
            raise RuntimeError("canonical target_bin_default is not unique")
        xyz = (matches[0].get("pose") or {}).get("xyz")
        if (not isinstance(xyz, list) or len(xyz) != 3
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in xyz)):
            raise RuntimeError("canonical target_bin_default pose is invalid")
        return {"id": "default_drop_zone", "target_id": "target_bin_default",
                "frame_id": "world", "pose_xyz": [float(value) for value in xyz]}
    cell = yaml.safe_load(cell_path.read_text(encoding="utf-8")) or {}
    task = cell.get("task") or {}
    target_id = destination_zone or str((task.get("place") or {}).get("target_ref") or "")
    zones = ((cell.get("environment") or {}).get("task_zones")
             if isinstance(cell.get("environment"), dict) else []) or []
    zone = next((item for item in zones if str(item.get("id")) == target_id), None)
    if not isinstance(zone, dict):
        raise RuntimeError(f"authored destination zone is missing: {target_id}")
    xyz = zone.get("pose_xyz") or (zone.get("pose") or {}).get("xyz")
    dimensions = zone.get("dimensions")
    if (not isinstance(xyz, list) or len(xyz) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz)):
        raise RuntimeError("authored destination zone pose is invalid")
    if (not isinstance(dimensions, list) or len(dimensions) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in dimensions)):
        raise RuntimeError("authored destination zone dimensions are invalid")
    destination = next((item for item in task.get("destinations", [])
                        if str(item.get("id")) == target_id), {})
    return {"id": target_id,
            "target_id": str(destination.get("target_ref") or zone.get("target_ref") or ""),
            "frame_id": str(zone.get("frame") or "world"),
            "pose_xyz": [float(v) for v in xyz],
            "dimensions": [float(v) for v in dimensions],
            "pose_rpy": [float(v) for v in (zone.get("pose_rpy") or [0.0, 0.0, 0.0])]}


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


class CandidateFailure(RuntimeError):
    def __init__(self, stage, reason):
        super().__init__(reason)
        self.stage = stage


def choose_cycle(targets, indices, preplan, attempts):
    """First fully feasible pair in confidence/id then preferred-grasp order."""
    for target in sorted(targets, key=lambda o: (-o['confidence'], o['id'])):
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
    parser.add_argument('--task-request', required=True)
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
    rclpy.init()
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
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
            summary['cancellation_confirmed'] = bool(cancel.done() and cancel.result() and cancel.result().goals_canceling)
            raise
        response = future.result()
        if response.status != 4 or response.result.error_code.val != 1:
            raise RuntimeError(f'MoveIt action failed: status={response.status}, code={response.result.error_code.val}')
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
    def plan_segment(view, name, goal, group=None, straight=False):
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
        if straight:
            # Chain short collision-planned moves in the same private scene.
            # Validate FK along each returned trajectory, rather than treating
            # endpoint feasibility as proof of a straight collision-safe path.
            start_pose = fk(view.robot_state, contract['tool_link'])
            a, b = pose_values(start_pose.pose), pose_values(goal.pose)
            if math.dist(a[:2], b[:2]) > 0.002:
                raise RuntimeError('contact/retreat requires a vertical path')
            count = max(1, math.ceil(math.dist(a[:3], b[:3]) / 0.005))
            combined = None
            elapsed_ns = 0
            total_planning_time = 0.0
            for i in range(1, count+1):
                waypoint = pose_message([x+(y-x)*i/count for x,y in zip(a[:3],b[:3])] + b[3:])
                part = plan_segment(view, name, waypoint, group)
                trajectory = part['trajectory']
                for point in trajectory.joint_trajectory.points:
                    sample = updated_state(view.robot_state, dict(zip(trajectory.joint_trajectory.joint_names, point.positions)), mimics)
                    actual_pose = pose_values(fk(sample, contract['tool_link']).pose)
                    if (math.dist(actual_pose[:2], b[:2]) > 0.0025 or
                            not min(a[2],b[2])-0.001 <= actual_pose[2] <= max(a[2],b[2])+0.001 or
                            min(math.dist(actual_pose[3:],b[3:]),math.dist(actual_pose[3:],[-q for q in b[3:]])) > 0.005):
                        raise RuntimeError('planned contact/retreat path leaves the Cartesian corridor')
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
                    attached_ids=[o.object.id for o in before.robot_state.attached_collision_objects],
                    world_ids=[o.id for o in before.world.collision_objects]))
        if not isinstance(goal, dict):
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = contract['planning_group']
            ik_request.ik_request.ik_link_name = contract['tool_link']
            ik_request.ik_request.pose_stamped = goal
            ik_request.ik_request.robot_state = view.robot_state
            # Seed continuity only. Collision feasibility is established by
            # MoveGroup's private-scene plan, never by this IK result alone.
            ik_request.ik_request.avoid_collisions = False
            ik_request.ik_request.timeout.sec = 1
            ik = call(ik_client, ik_request)
            if ik.error_code.val != 1:
                raise RuntimeError(f'IK failed: {ik.error_code.val}')
            values = dict(zip(ik.solution.joint_state.name,ik.solution.joint_state.position))
            request.goal_constraints = [joint_constraints({n:values[n] for n in contract['home_joint_names']})]
        goal_msg = MoveGroup.Goal(request=request)
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.planning_scene_diff = copy.deepcopy(view)
        result = action(plan_client, goal_msg, 12)
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
        task = inputs.task_request(yaml.safe_load(Path(args.task_request).read_text()), cell)
        stage('ACQUIRE_OBJECTS')
        snapshot = yaml.safe_load(Path(args.detections).read_text())
        if args.replay:
            snapshot = inputs.replay_snapshot(snapshot, time.time())
        objects = inputs.normalize(snapshot, time.time(), _PLANNER)
        stage('FILTER_TARGETS')
        eligible, rejected = inputs.filter_targets(objects, task, cell, time.time(), _PLANNER)
        summary.update(task_request=task, normalized_objects=objects, rejected_objects=rejected)
        params = call(params_client, GetParameters.Request(names=['use_fake_hardware','allow_trajectory_execution','robot_description'])).values
        summary['fake_hardware_guard'] = fake_hardware_evidence(params, call(hardware_client,ListHardwareComponents.Request()).component)
        if args.start and not params[1].bool_value:
            raise RuntimeError('fake execution disabled; launch allow_trajectory_execution:=true')
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
        def preplan(target, index, record):
            steps = []
            view = copy.deepcopy(initial)
            def motion(name, goal, group=None, straight=False):
                nonlocal view
                step = plan_segment(view,name,goal,group,straight)
                steps.append(step)
                record['stages'].append(step['metadata'])
                view = copy.deepcopy(step['after'])
                return view
            try:
                if time.monotonic() > deadline:
                    raise RuntimeError('candidate search budget exhausted')
                stage('GENERATE_GRASPS')
                if time.time()-target['timestamp'] > task['max_age_seconds']:
                    raise RuntimeError('observation expired before candidate planning')
                geometry = build_grasp_target(target)
                extents = oriented_box_extents(geometry)
                if min(extents[:2]) > 0.085:
                    raise RuntimeError('target exceeds Robotiq aperture')
                if any(a>b for a,b in zip(extents,destination['dimensions'])):
                    raise RuntimeError('target exceeds destination bounds')
                original = next(o for o in initial.world.collision_objects if o.id==target['id'])
                approach = pose_message(_PLANNER.tool_pose_for_grasp(generate_box_grasp_candidates(geometry,contract['approach_distance_m'])[index],contract))
                contact = pose_message(_PLANNER.tool_pose_for_grasp(generate_box_grasp_candidates(geometry,0.0)[index],contract))
                motion('PREPLAN_APPROACH',approach)
                view.allowed_collision_matrix = target_contact_matrix(baseline,target['id'],contract['allowed_touch_links'])
                motion('PREPLAN_GRASP',contact,straight=True)
                stage('PREPLAN_CLOSE_GRIPPER')
                close = None
                # Live scene remains unchanged, so this validity query evaluates
                # the predicted un-attached grasp against the original obstacles.
                for i in range(1,81):
                    trial = updated_state(view.robot_state,{'gripper_finger1_joint':0.804*i/80},mimics)
                    response = call(validity_client,GetStateValidity.Request(robot_state=trial,group_name=''))
                    if response.contacts:
                        verify_selected_contacts(response.contacts,target['id'],contract['allowed_touch_links'])
                        close = 0.804*i/80
                        break
                if close is None:
                    raise RuntimeError('no allowed fingertip contact in closing range')
                motion('PREPLAN_CLOSE_GRIPPER',{'gripper_finger1_joint':close},group='gripper')
                tool_at_grasp = fk(view.robot_state,contract['tool_link'])
                frame_at_grasp = fk(view.robot_state,contract['grasp_frame'])
                before = copy.deepcopy(view)
                view = private_attachment(view,original,contract['grasp_frame'],frame_at_grasp.pose,contract['allowed_touch_links'])
                view.allowed_collision_matrix = copy.deepcopy(baseline)
                steps.append(dict(kind='attach',stage='ATTACH',before=before,after=copy.deepcopy(view),original=original))
                motion('PREPLAN_LIFT',translated_pose(tool_at_grasp,dz=retreat))
                delta = [a-b for a,b in zip(destination['pose_xyz'],target['pose'][:3])]
                motion('PREPLAN_TRANSFER',translated_pose(tool_at_grasp,*[delta[0],delta[1],delta[2]+retreat]))
                motion('PREPLAN_PLACE',translated_pose(tool_at_grasp,*delta))
                reached = fk(view.robot_state,contract['tool_link'])
                achieved = object_pose_after_motion(original,tool_at_grasp.pose,reached.pose)
                if math.dist(achieved[:3],destination['pose_xyz']) > 0.003:
                    raise RuntimeError('planned placement differs from destination by more than 3 mm')
                motion('PREPLAN_OPEN_GRIPPER',{'gripper_finger1_joint':0.0},group='gripper')
                before = copy.deepcopy(view)
                placed = place_detachment_diff(original,contract['grasp_frame'],achieved[:3],achieved[3:]).world.collision_objects[0]
                view = copy.deepcopy(view)
                view.robot_state.attached_collision_objects = []
                view.world.collision_objects.append(placed)
                steps.append(dict(kind='detach',stage='DETACH',before=before,after=copy.deepcopy(view),original=original,tool_at_grasp=tool_at_grasp))
                view.allowed_collision_matrix = target_contact_matrix(baseline,target['id'],contract['allowed_touch_links'])
                motion('PREPLAN_RETREAT',translated_pose(reached,dz=retreat),straight=True)
                view.allowed_collision_matrix = copy.deepcopy(baseline)
                motion('PREPLAN_HOME',home)
                stage('CANDIDATE_READY')
                return dict(object_id=target['id'],grasp_index=index,steps=steps,full_cycle_prevalidated=True)
            except Exception as exc:
                record['stages'].append(dict(stage=summary['current_stage'],success=False,reason=str(exc)))
                raise CandidateFailure(summary['current_stage'],str(exc)) from exc
        stage('ENUMERATE_TARGETS')
        cycle = choose_cycle(eligible,candidate_indices(8),preplan,summary['candidate_attempts'])
        selected_id = cycle['object_id']
        summary.update(selected_object_id=selected_id,selected_grasp_index=cycle['grasp_index'],full_cycle_prevalidated=True,
                       full_cycle_plan_success=True,plan_metadata=[s['metadata'] for s in cycle['steps'] if s['kind']=='motion'])
        stage('VERIFY_PREPLAN_UNCHANGED')
        assert_scene_match(scene_now(),initial)
        summary['prevalidation_left_live_scene_unchanged'] = True
        if not args.start:
            summary['result'] = 'PLAN_ONLY'
            return 0
        fresh, _ = inputs.filter_targets(objects,task,cell,time.time(),_PLANNER)
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
                summary.setdefault('execution_results',[]).append(dict(stage=label,code=result.error_code.val))
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
            assert_scene_match(scene_now(),expected,selected_id)
            summary['stages'].append(label)
        final = scene_now()
        summary['final_planning_scene'] = dict(world_ids=[o.id for o in final.world.collision_objects],
            attached_ids=[o.object.id for o in final.robot_state.attached_collision_objects],distractors_unchanged=True)
        stage('COMPLETE')
        summary.update(result='PASS',full_cycle_execution_success=True)
    except (Exception,KeyboardInterrupt) as exc:
        summary.update(failed_stage=exc.stage if isinstance(exc,CandidateFailure) else summary['current_stage'],failure=str(exc))
        stage('FAILED')
        summary['recovery_required'] = summary['execution_attempted']
        # Do not erase held/placed objects or command a recovery trajectory.
    finally:
        if summary['execution_attempted'] and summary['result'] != 'PASS':
            try:
                apply(PlanningScene(is_diff=True,allowed_collision_matrix=baseline))
                recovery = scene_now()
                summary['recovery_scene'] = dict(
                    attached_ids=[o.object.id for o in recovery.robot_state.attached_collision_objects],
                    world_ids=[o.id for o in recovery.world.collision_objects],
                    contact_acm_restored=collision_matrix_signature(recovery.allowed_collision_matrix)==collision_matrix_signature(baseline))
            except Exception as recovery_error:
                summary['recovery_inspection_failure'] = str(recovery_error)
        Path(args.summary_output).write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
        print(json.dumps(summary,indent=2,sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary['result']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
