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


def load_canonical_place_target(package_share):
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
    target_id = str((task.get("place") or {}).get("target_ref") or "")
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary-output", required=True)
    parser.add_argument("--scene-package", default="ur5_2f_test")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--retreat-distance", type=float, default=None)
    args = parser.parse_args()

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK, GetCartesianPath, GetStateValidity
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from controller_manager_msgs.srv import ListHardwareComponents
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node("perceived_object_grasp_execute")
    summary = {"result": "FAIL", "execution_attempted": False,
               "real_hardware": None, "cleanup_performed": False}
    selected_id = None
    summary["stages"] = []
    attached = False

    def service(service_type, name):
        return node.create_client(service_type, name)

    scene_client = service(GetPlanningScene, "/get_planning_scene")
    apply_client = service(ApplyPlanningScene, "/apply_planning_scene")
    ik_client = service(GetPositionIK, "/compute_ik")
    fk_client = service(GetPositionFK, "/compute_fk")
    plan_client = service(GetMotionPlan, "/plan_kinematic_path")
    cartesian_client = service(GetCartesianPath, "/compute_cartesian_path")
    validity_client = service(GetStateValidity, "/check_state_validity")
    parameter_client = service(GetParameters, "/move_group/get_parameters")
    ownership_client = service(
        SetParameters, "/epd_dynamic_planning_scene_bridge/set_parameters")
    hardware_client = service(ListHardwareComponents, "/controller_manager/list_hardware_components")
    execute_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    arm_controller_client = ActionClient(
        node, FollowJointTrajectory, "/ur5_arm_controller/follow_joint_trajectory")

    def call(client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service timed out: {client.srv_name}")
        return future.result()

    def get_scene(components):
        request = GetPlanningScene.Request()
        request.components.components = components
        return call(scene_client, request).scene

    def set_ownership(object_id):
        parameter = Parameter()
        parameter.name = "owned_object_id"
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=object_id)
        response = call(
            ownership_client, SetParameters.Request(parameters=[parameter]), timeout=5.0)
        if len(response.results) != 1 or not response.results[0].successful:
            raise RuntimeError("perception bridge rejected manipulation ownership")
        return object_id

    def plan_to_joint_state(start_state, joint_state):
        goal = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            if any(name.endswith(suffix) for suffix in ARM_JOINT_SUFFIXES):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = position
                constraint.tolerance_above = 0.001
                constraint.tolerance_below = 0.001
                constraint.weight = 1.0
                goal.joint_constraints.append(constraint)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = grasp_contract["planning_group"]
        motion.start_state = copy.deepcopy(start_state)
        motion.start_state.is_diff = True
        motion.goal_constraints = [goal]
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2
        return call(plan_client, request, timeout=12.0).motion_plan_response

    def apply_matrix(matrix):
        from moveit_msgs.msg import PlanningScene
        diff = PlanningScene(is_diff=True, allowed_collision_matrix=matrix)
        if not call(apply_client, ApplyPlanningScene.Request(scene=diff)).success:
            raise RuntimeError("PlanningScene rejected contact ACM update/restore")
        actual = get_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        if collision_matrix_signature(actual) != collision_matrix_signature(matrix):
            raise RuntimeError("PlanningScene contact ACM verification failed")

    def validate_state(scene, label):
        result = call(validity_client, GetStateValidity.Request(
            robot_state=scene.robot_state, group_name=""))
        if not result.valid:
            contacts = [{"a": c.contact_body_1, "b": c.contact_body_2,
                         "depth_m": c.depth} for c in result.contacts]
            summary["blocked_contacts"] = contacts
            raise RuntimeError(f"{label} state is in collision: {contacts}")

    def execute(trajectory, label):
        if not execute_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveIt execute_trajectory action unavailable")
        summary["execution_attempted"] = True
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        sent = execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} execution goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
        if not result_future.done() or result_future.result() is None:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
            raise RuntimeError(f"{label} execution timed out")
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} execution failed: {result.error_code.val}")
        summary["stages"].append(label)
        return result.error_code.val

    def plan_pose(start_scene, pose, label):
        request = GetPositionIK.Request()
        request.ik_request.group_name = grasp_contract["planning_group"]
        request.ik_request.ik_link_name = grasp_contract["tool_link"]
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = start_scene.robot_state
        # Select a collision-free IK branch before constraining the
        # authoritative motion plan.  OMPL still validates the complete motion
        # against the current PlanningScene (including the attached object).
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 3
        ik = call(ik_client, request, timeout=6.0)
        if ik.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} IK failed: {ik.error_code.val}")
        plan = plan_to_joint_state(start_scene.robot_state, ik.solution.joint_state)
        points = len(plan.trajectory.joint_trajectory.points)
        if plan.error_code.val != MoveItErrorCodes.SUCCESS or not points:
            raise RuntimeError(f"{label} planning failed: {plan.error_code.val}")
        return plan, points

    try:
        scene_package = Path(args.scene_package)
        if not scene_package.is_dir():
            scene_package = Path(get_package_share_directory(args.scene_package))
        grasp_contract = _PLANNER.load_grasp_contract(str(scene_package))
        retreat_distance = (float(args.retreat_distance)
                            if args.retreat_distance is not None
                            else float(grasp_contract["retreat_distance_m"]))
        param_request = GetParameters.Request(names=["use_fake_hardware"])
        params = call(parameter_client, param_request).values
        hardware = call(hardware_client, ListHardwareComponents.Request()).component
        summary["fake_hardware_guard"] = fake_hardware_evidence(params, hardware)
        summary["real_hardware"] = False
        execution_setting = call(parameter_client, GetParameters.Request(
            names=["allow_trajectory_execution"])).values
        if not execution_setting or not execution_setting[0].bool_value:
            raise RuntimeError("fake trajectory execution is disabled; launch with allow_trajectory_execution:=true")
        summary["arm_controller_action_available"] = arm_controller_client.wait_for_server(
            timeout_sec=2.0)
        if not summary["arm_controller_action_available"]:
            raise RuntimeError("fake arm controller action unavailable")

        initial_scene = get_scene(PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        validate_state(initial_scene, "initial")
        summary["initial_state_valid"] = True
        from sensor_msgs.msg import JointState
        home = JointState(name=grasp_contract["home_joint_names"], position=[float(v) for v in grasp_contract["home_joint_positions"]])
        current = dict(zip(initial_scene.robot_state.joint_state.name, initial_scene.robot_state.joint_state.position))
        if any(abs(current[name] - position) > 0.001 for name, position in zip(home.name, home.position)):
            home_plan = plan_to_joint_state(initial_scene.robot_state, home)
            if home_plan.error_code.val != MoveItErrorCodes.SUCCESS or not home_plan.trajectory.joint_trajectory.points:
                raise RuntimeError(f"HOME planning failed: {home_plan.error_code.val}")
            execute(home_plan.trajectory, "HOME")
        else:
            summary["stages"].append("HOME")
        deadline = time.monotonic() + args.timeout
        original = None
        scene = None
        while time.monotonic() < deadline:
            components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                          | PlanningSceneComponents.ROBOT_STATE)
            scene = get_scene(components)
            perceived = [(obj, collision_object_dict(obj))
                         for obj in scene.world.collision_objects
                         if str(obj.id).startswith("epd::") or str(obj.id).isdigit()]
            valid = [(obj, item) for obj, item in perceived if item]
            try:
                chosen = select_graspable_box([item for _, item in valid])
                original = next(obj for obj, item in valid if item["id"] == chosen["id"])
                break
            except ValueError as exc:
                if valid:
                    raise RuntimeError(f"live objects rejected: {exc}; dimensions="
                                       f"{[item['dimensions'] for _, item in valid]}") from exc
                time.sleep(0.1)
        if original is None:
            raise RuntimeError("timed out waiting for one valid live perceived object")

        target = build_grasp_target(chosen)
        selected_id = target["perceived_object_id"]
        summary["ownership_claimed_id"] = set_ownership(selected_id)
        # Refresh after the bridge acknowledges ownership so the selected
        # geometry and the subsequent plans use the same protected version.
        scene = get_scene(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE)
        matches = [obj for obj in scene.world.collision_objects if obj.id == selected_id]
        if len(matches) != 1:
            raise RuntimeError("owned target disappeared or is not unique")
        original = matches[0]
        target = build_grasp_target(collision_object_dict(original))
        candidates = [_PLANNER.tool_pose_for_grasp(values, grasp_contract) for values in
                      generate_box_grasp_candidates(target, clearance=grasp_contract["approach_distance_m"])]
        contact_candidates = [_PLANNER.tool_pose_for_grasp(values, grasp_contract) for values in
                              generate_box_grasp_candidates(target, clearance=0.0)]
        touch_links = grasp_contract["allowed_touch_links"]
        validate_state(scene, "approach start")
        ids = [obj.id for obj in scene.world.collision_objects
               if str(obj.id).startswith("epd::") or str(obj.id).isdigit()]
        place_target = load_canonical_place_target(scene_package)
        live_extents = oriented_box_extents(target)
        if any(extent > limit + 1e-6
               for extent, limit in zip(live_extents, place_target["dimensions"])):
            raise RuntimeError(
                "live perceived box does not fit the authored destination region "
                f"without a commanded reorientation: extents={live_extents}, "
                f"region={place_target['dimensions']}")
        summary.update({
            "selected_object_id": selected_id,
            "object_frame": target["planning_frame"],
            "object_pose": target["target_pose"],
            "object_dimensions": target["target_dimensions"],
            "approach_distance_m": grasp_contract["approach_distance_m"],
            "retreat_distance_m": retreat_distance,
            "geometry_valid": True,
            "grasp_candidates_generated": len(candidates),
            "planning_scene_selected_id_count": ids.count(selected_id),
            "duplicate_ids": sorted({item for item in ids if ids.count(item) > 1}),
            "place_target": place_target,
        })
        if ids.count(selected_id) != 1 or summary["duplicate_ids"]:
            raise RuntimeError("perceived object identity is not unique")

        grasp_plan = None
        for index in candidate_indices(len(candidates)):
            values = candidates[index]
            pose = PoseStamped()
            pose.header.frame_id = target["planning_frame"]
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
             pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = values
            request = GetPositionIK.Request()
            request.ik_request.group_name = grasp_contract["planning_group"]
            request.ik_request.ik_link_name = grasp_contract["tool_link"]
            request.ik_request.pose_stamped = pose
            request.ik_request.robot_state = scene.robot_state
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 2
            ik = call(ik_client, request, timeout=5.0)
            if ik.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            plan = plan_to_joint_state(scene.robot_state, ik.solution.joint_state)
            points = len(plan.trajectory.joint_trajectory.points)
            if plan.error_code.val == MoveItErrorCodes.SUCCESS and points:
                grasp_plan = plan
                summary.update({"successful_candidate_index": index,
                                "grasp_moveit_error_code": plan.error_code.val,
                                "grasp_trajectory_point_count": points})
                break
        if grasp_plan is None:
            raise RuntimeError("no grasp candidate produced a non-empty MoveIt plan")

        summary["grasp_execution_error_code"] = execute(grasp_plan.trajectory, "APPROACH")
        contact_pose = copy.deepcopy(pose)
        values = contact_candidates[summary["successful_candidate_index"]]
        (contact_pose.pose.position.x, contact_pose.pose.position.y, contact_pose.pose.position.z,
         contact_pose.pose.orientation.x, contact_pose.pose.orientation.y,
         contact_pose.pose.orientation.z, contact_pose.pose.orientation.w) = values
        baseline = get_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        with temporary_target_contact(baseline, selected_id, touch_links, apply_matrix):
            contact_scene = get_scene(PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
            if not any(obj.id == selected_id for obj in contact_scene.world.collision_objects):
                raise RuntimeError("selected target disappeared before contact")
            request = GetCartesianPath.Request()
            request.header = contact_pose.header
            request.start_state = contact_scene.robot_state
            request.start_state.is_diff = True
            request.group_name = grasp_contract["planning_group"]
            request.link_name = grasp_contract["tool_link"]
            request.waypoints = [contact_pose.pose]
            request.max_step = 0.005
            request.revolute_jump_threshold = 0.2
            request.avoid_collisions = True
            contact = call(cartesian_client, request)
            summary["contact_fraction"] = contact.fraction
            summary["contact_allowed_touch_links"] = touch_links
            if contact.error_code.val != MoveItErrorCodes.SUCCESS or contact.fraction < 1.0 or not contact.solution.joint_trajectory.points:
                raise RuntimeError(f"GRASP CONTACT incomplete: code={contact.error_code.val}, fraction={contact.fraction}")
            execute(contact.solution, "GRASP CONTACT")
            # Restore normal rules briefly, with the arm stopped, to measure
            # which links actually contact the target before attaching it.
            apply_matrix(baseline)
            measured_scene = get_scene(PlanningSceneComponents.ROBOT_STATE)
            measured = call(validity_client, GetStateValidity.Request(
                robot_state=measured_scene.robot_state, group_name=""))
            summary["grasp_contacts"] = [{"a": c.contact_body_1, "b": c.contact_body_2,
                                          "depth_m": c.depth} for c in measured.contacts]
            verify_selected_contacts(measured.contacts, selected_id, touch_links)
            pre_attach_scene = get_scene(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
            if not any(obj.id == selected_id for obj in pre_attach_scene.world.collision_objects):
                raise RuntimeError("selected target disappeared before attachment")
            diff = attachment_diff(original, grasp_contract["grasp_frame"], touch_links)
            attached = True  # Retain ownership if the service response is lost.
            if not call(apply_client, ApplyPlanningScene.Request(scene=diff)).success:
                raise RuntimeError("PlanningScene rejected object attachment")
            verify_components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                 | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                                 | PlanningSceneComponents.ROBOT_STATE)
            attached_scene = get_scene(verify_components)
            summary["attachment"] = attachment_status(attached_scene, selected_id, grasp_contract["grasp_frame"])
            if not summary["attachment"]["valid"]:
                raise RuntimeError("attachment verification failed")
            summary["stages"].append("ATTACH")
        summary["contact_acm_restored"] = True
        attached_scene = get_scene(verify_components)
        validate_state(attached_scene, "attached")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = target["planning_frame"]
        fk_request.fk_link_names = [grasp_contract["tool_link"]]
        fk_request.robot_state = attached_scene.robot_state
        fk = call(fk_client, fk_request)
        if fk.error_code.val != MoveItErrorCodes.SUCCESS or len(fk.pose_stamped) != 1:
            raise RuntimeError(f"retreat FK failed: {fk.error_code.val}")
        current_tool_pose = copy.deepcopy(fk.pose_stamped[0])
        lift_pose = translated_pose(current_tool_pose, dz=retreat_distance)
        lift_plan, lift_points = plan_pose(attached_scene, lift_pose, "attached lift")
        summary.update({"attached_lift_moveit_error_code": lift_plan.error_code.val,
                        "attached_lift_trajectory_point_count": lift_points,
                        "attached_lift_execution_error_code": execute(
                            lift_plan.trajectory, "RETREAT_ATTACHED")})
        lifted_scene = get_scene(verify_components)

        place_target = summary["place_target"]["pose_xyz"]
        place_dx = place_target[0] - target["target_pose"][0]
        place_dy = place_target[1] - target["target_pose"][1]
        place_dz = place_target[2] - target["target_pose"][2]
        above_place_pose = translated_pose(
            current_tool_pose, place_dx, place_dy,
            place_dz + retreat_distance)
        transfer_plan, transfer_points = plan_pose(
            lifted_scene, above_place_pose, "above-place transfer")
        summary.update({"transfer_moveit_error_code": transfer_plan.error_code.val,
                        "transfer_trajectory_point_count": transfer_points,
                        "transfer_execution_error_code": execute(
                            transfer_plan.trajectory, "TRANSFER")})
        above_place_scene = get_scene(verify_components)

        place_pose = translated_pose(current_tool_pose, place_dx, place_dy, place_dz)
        place_plan, place_points = plan_pose(above_place_scene, place_pose, "place")
        summary.update({"place_moveit_error_code": place_plan.error_code.val,
                        "place_trajectory_point_count": place_points,
                        "place_execution_error_code": execute(place_plan.trajectory, "PLACE")})

        # Detach at the achieved FK pose, never teleport to the ideal goal.
        at_place = get_scene(verify_components)
        validate_state(at_place, "place")
        fk_request.robot_state = at_place.robot_state
        reached = call(fk_client, fk_request)
        if reached.error_code.val != MoveItErrorCodes.SUCCESS or len(reached.pose_stamped) != 1:
            raise RuntimeError("place FK unavailable")
        achieved = object_pose_after_motion(original, current_tool_pose.pose, reached.pose_stamped[0].pose)
        place_diff = place_detachment_diff(original, grasp_contract["grasp_frame"], achieved[:3], achieved[3:])
        placed_geometry = collision_object_dict(place_diff.world.collision_objects[0])
        summary["achieved_place_pose"] = placed_geometry["pose"]
        if math.dist(placed_geometry["pose"][:3], place_target) > 0.005:
            raise RuntimeError("achieved attached-object position differs from the authored place goal by more than 5 mm")
        if not call(apply_client, ApplyPlanningScene.Request(scene=place_diff)).success:
            raise RuntimeError("PlanningScene rejected place detachment")
        attached = False
        placed_scene = get_scene(verify_components)
        placed_status = attachment_status(placed_scene, selected_id, grasp_contract["grasp_frame"])
        placed_world = [obj for obj in placed_scene.world.collision_objects
                        if obj.id == selected_id]
        summary["detachment"] = {
            "attached_object_present": placed_status["attached_object_present"],
            "world_object_count": len(placed_world),
            "world_object_present": len(placed_world) == 1,
        }
        if placed_status["attached_object_present"] or len(placed_world) != 1:
            raise RuntimeError("place detachment verification failed")
        summary["stages"].append("DETACH")

        retreat_pose = translated_pose(reached.pose_stamped[0], dz=retreat_distance)
        # Fingertips can initially remain in contact after detachment. Only
        # allow those pairs during a straight departure, then restore normal
        # collision rules and verify the separated state before returning home.
        release_baseline = get_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
        with temporary_target_contact(release_baseline, selected_id, touch_links, apply_matrix):
            request = GetCartesianPath.Request()
            request.header = retreat_pose.header
            request.start_state = placed_scene.robot_state
            request.start_state.is_diff = True
            request.group_name = grasp_contract["planning_group"]
            request.link_name = grasp_contract["tool_link"]
            request.waypoints = [retreat_pose.pose]
            request.max_step = 0.005
            request.revolute_jump_threshold = 0.2
            request.avoid_collisions = True
            departure = call(cartesian_client, request)
            if departure.error_code.val != MoveItErrorCodes.SUCCESS or departure.fraction < 1.0 or not departure.solution.joint_trajectory.points:
                raise RuntimeError(f"place retreat incomplete: code={departure.error_code.val}, fraction={departure.fraction}")
            execute(departure.solution, "RETREAT_AFTER_PLACE")
        summary["release_acm_restored"] = True
        final_scene = get_scene(verify_components)
        validate_state(final_scene, "place retreat")
        final_world = [obj for obj in final_scene.world.collision_objects
                       if obj.id == selected_id]
        final_attached = [item for item in final_scene.robot_state.attached_collision_objects
                          if item.object.id == selected_id]
        summary["final_planning_scene"] = {
            "world_object_count": len(final_world),
            "attached_object_count": len(final_attached),
            "duplicate_ids": len(final_world) > 1,
            "valid": len(final_world) == 1 and not final_attached,
        }
        if not summary["final_planning_scene"]["valid"]:
            raise RuntimeError("final PlanningScene verification failed")
        home_plan = plan_to_joint_state(final_scene.robot_state, home)
        if home_plan.error_code.val != MoveItErrorCodes.SUCCESS or not home_plan.trajectory.joint_trajectory.points:
            raise RuntimeError(f"return HOME planning failed: {home_plan.error_code.val}")
        execute(home_plan.trajectory, "HOME")
        validate_state(get_scene(verify_components), "final HOME")
        summary["result"] = "PASS"
    except Exception as exc:
        summary["failure"] = str(exc)
        if attached and selected_id:
            # A failed transfer does not make the held object disappear.
            # Preserve its actual attachment and bridge ownership for recovery.
            summary["attachment_retained_for_recovery"] = selected_id
    finally:
        retain_ownership = attached or "detachment" in summary
        if retain_ownership:
            # Fake execution does not physically move the camera-observed
            # object. Keep its ID protected until explicit scene recovery/reset
            # so live observations cannot teleport the simulated object back.
            summary["ownership_retained_for_scene_recovery"] = selected_id
        if summary.get("ownership_claimed_id") and not summary.get("ownership_released") and not retain_ownership:
            try:
                set_ownership("")
                summary["ownership_released"] = True
            except Exception as ownership_exc:
                summary["ownership_release_failure"] = str(ownership_exc)
        Path(args.summary_output).write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
