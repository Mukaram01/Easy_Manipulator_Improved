#!/usr/bin/env python3
"""Plan, execute, attach, and retreat for one perceived object on fake hardware."""

import argparse
import copy
import importlib.util
import json
import math
import time
from pathlib import Path


_PLANNER_PATH = Path(__file__).with_name("perceived_object_grasp_plan.py")
_SPEC = importlib.util.spec_from_file_location("p8e1_grasp_plan", _PLANNER_PATH)
_PLANNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_PLANNER)

ARM_JOINT_SUFFIXES = _PLANNER.ARM_JOINT_SUFFIXES
select_perceived_box = _PLANNER.select_perceived_box
build_grasp_target = _PLANNER.build_grasp_target
generate_box_grasp_candidates = _PLANNER.generate_box_grasp_candidates
collision_object_dict = _PLANNER._collision_object_dict


def select_graspable_box(objects, max_aperture=0.085, max_planar_extent=0.20):
    """Select deterministically among boxes that physically fit the 2F gripper."""
    feasible = [obj for obj in objects
                if min(obj.get("dimensions", [math.inf])[:2]) <= max_aperture
                and max(obj.get("dimensions", [math.inf])[:2]) <= max_planar_extent]
    if not feasible:
        raise ValueError("no perceived BOX fits the Robotiq 2F grasp envelope")
    return select_perceived_box(feasible)


def support_penetration_correction(selected, objects, clearance=0.001):
    """Lift a supported box only enough to remove proxy penetration."""
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

    layout_path = Path(package_share) / "layout" / "workcell_studio_layout.yaml"
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    matches = [item for item in layout.get("items", [])
               if item.get("id") == "target_bin_default"]
    if len(matches) != 1:
        raise RuntimeError("canonical target_bin_default is not unique")
    xyz = matches[0].get("pose", {}).get("xyz")
    if not isinstance(xyz, list) or len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        raise RuntimeError("canonical target_bin_default pose is invalid")
    return {"id": "default_drop_zone", "target_id": "target_bin_default",
            "frame_id": "world", "pose_xyz": [float(v) for v in xyz]}


def fake_hardware_evidence(parameter_values, hardware_components):
    """Fail closed unless MoveIt and controller_manager both prove mock hardware."""
    fake_parameter = bool(parameter_values and parameter_values[0].bool_value)
    classes = [component.class_type for component in hardware_components]
    mock_classes = [name for name in classes if "mock_components/GenericSystem" in name]
    real_markers = ("URPositionHardwareInterface", "ur_robot_driver", "EtherCAT", "RealSystem")
    real_classes = [name for name in classes if any(marker.lower() in name.lower()
                                                      for marker in real_markers)]
    if not fake_parameter or not mock_classes or real_classes:
        raise RuntimeError(
            "execution rejected: fake hardware was not exclusively proven active "
            f"(move_group={fake_parameter}, classes={classes})"
        )
    return {"move_group_use_fake_hardware": True,
            "hardware_classes": classes, "real_hardware": False}


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


def place_detachment_diff(original, link_name, place_xyz):
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
    if (placed.pose.orientation.x ** 2 + placed.pose.orientation.y ** 2
            + placed.pose.orientation.z ** 2 + placed.pose.orientation.w ** 2) < 1e-12:
        placed.pose.orientation.w = 1.0
    scene.world.collision_objects.append(placed)
    return scene


def translated_pose(pose, dx=0.0, dy=0.0, dz=0.0):
    """Return a copied tool pose translated in the planning frame."""
    result = copy.deepcopy(pose)
    result.pose.position.x += dx
    result.pose.position.y += dy
    result.pose.position.z += dz
    return result


def candidate_indices(count, preferred=3):
    """Try the live-proven retreat-capable orientation, then every fallback."""
    if not 0 <= preferred < count:
        return list(range(count))
    return [preferred] + [index for index in range(count) if index != preferred]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary-output", required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--retreat-distance", type=float, default=0.10)
    args = parser.parse_args()

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetPositionFK, GetPositionIK
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from controller_manager_msgs.srv import ListHardwareComponents
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node("perceived_object_grasp_execute")
    summary = {"result": "FAIL", "execution_attempted": False,
               "real_hardware": None, "cleanup_performed": False}
    selected_id = None
    attached = False

    def service(service_type, name):
        return node.create_client(service_type, name)

    scene_client = service(GetPlanningScene, "/get_planning_scene")
    apply_client = service(ApplyPlanningScene, "/apply_planning_scene")
    ik_client = service(GetPositionIK, "/compute_ik")
    fk_client = service(GetPositionFK, "/compute_fk")
    plan_client = service(GetMotionPlan, "/plan_kinematic_path")
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
        motion.group_name = "manipulator"
        motion.start_state = copy.deepcopy(start_state)
        motion.start_state.is_diff = True
        motion.goal_constraints = [goal]
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2
        return call(plan_client, request, timeout=12.0).motion_plan_response

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
            raise RuntimeError(f"{label} execution timed out")
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} execution failed: {result.error_code.val}")
        return result.error_code.val

    def plan_pose(start_scene, pose, label):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "manipulator"
        request.ik_request.ik_link_name = "tool0"
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = start_scene.robot_state
        # Match the established grasp path: obtain a geometric IK seed, then
        # require OMPL to validate the complete attached-object motion.
        request.ik_request.avoid_collisions = False
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
        param_request = GetParameters.Request(names=["use_fake_hardware"])
        params = call(parameter_client, param_request).values
        hardware = call(hardware_client, ListHardwareComponents.Request()).component
        summary["fake_hardware_guard"] = fake_hardware_evidence(params, hardware)
        summary["real_hardware"] = False
        summary["arm_controller_action_available"] = arm_controller_client.wait_for_server(
            timeout_sec=2.0)
        if not summary["arm_controller_action_available"]:
            raise RuntimeError("fake arm controller action unavailable")

        deadline = time.monotonic() + args.timeout
        original = None
        scene = None
        while time.monotonic() < deadline:
            components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                          | PlanningSceneComponents.ROBOT_STATE)
            scene = get_scene(components)
            perceived = [(obj, collision_object_dict(obj))
                         for obj in scene.world.collision_objects if obj.id.isdigit()]
            valid = [(obj, item) for obj, item in perceived if item]
            try:
                chosen = select_graspable_box([item for _, item in valid])
                original = next(obj for obj, item in valid if item["id"] == chosen["id"])
                break
            except ValueError:
                time.sleep(0.1)
        if original is None:
            raise RuntimeError("timed out waiting for one valid live perceived object")

        correction = support_penetration_correction(
            chosen, [item for _, item in valid])
        if correction:
            correction_z, support_id = correction
            original = copy.deepcopy(original)
            original.pose.position.z += correction_z
            chosen = copy.deepcopy(chosen)
            chosen["pose"][2] += correction_z
            correction_scene = attachment_diff(original, "ee_palm", [], remove_world=False)
            correction_scene.robot_state.attached_collision_objects.clear()
            correction_scene.robot_state.is_diff = True
            correction_scene.world.collision_objects.append(copy.deepcopy(original))
            if not call(apply_client, ApplyPlanningScene.Request(scene=correction_scene)).success:
                raise RuntimeError("PlanningScene rejected support penetration correction")
            summary["support_penetration_correction"] = {
                "support_object_id": support_id, "z_translation": correction_z}

        target = build_grasp_target(chosen)
        selected_id = target["perceived_object_id"]
        summary["ownership_claimed_id"] = set_ownership(selected_id)
        candidates = generate_box_grasp_candidates(target, clearance=(0.32, 0.42))
        ids = [obj.id for obj in scene.world.collision_objects if obj.id.isdigit()]
        summary.update({
            "selected_object_id": selected_id,
            "object_frame": target["planning_frame"],
            "object_pose": target["target_pose"],
            "object_dimensions": target["target_dimensions"],
            "geometry_valid": True,
            "grasp_candidates_generated": len(candidates),
            "planning_scene_selected_id_count": ids.count(selected_id),
            "duplicate_ids": sorted({item for item in ids if ids.count(item) > 1}),
            "place_target": load_canonical_place_target(
                get_package_share_directory("ur5_2f_test")),
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
            request.ik_request.group_name = "manipulator"
            request.ik_request.ik_link_name = "tool0"
            request.ik_request.pose_stamped = pose
            request.ik_request.robot_state = scene.robot_state
            request.ik_request.avoid_collisions = False
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

        summary["grasp_execution_error_code"] = execute(grasp_plan.trajectory, "grasp")

        pre_attach_scene = get_scene(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        world_present_before_attach = any(
            obj.id == selected_id for obj in pre_attach_scene.world.collision_objects)
        summary["world_object_present_before_attach"] = world_present_before_attach
        diff = attachment_diff(original, "ee_palm", [
            "ee_palm", "tool0", "gripper_base_link",
            "gripper_finger1_finger_link", "gripper_finger2_finger_link",
            "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link",
        ], remove_world=world_present_before_attach)
        apply_request = ApplyPlanningScene.Request(scene=diff)
        attach_response = call(apply_client, apply_request)
        if not attach_response.success and world_present_before_attach:
            # LOST may race the pre-attach query. Retrying only the attachment
            # preserves the captured object truth without resurrecting it in world.
            summary["attachment_lifecycle_race_retried"] = True
            attach_only = attachment_diff(
                original, "ee_palm", diff.robot_state.attached_collision_objects[0].touch_links,
                remove_world=False)
            attach_response = call(
                apply_client, ApplyPlanningScene.Request(scene=attach_only))
        if not attach_response.success:
            raise RuntimeError("PlanningScene rejected object attachment")
        attached = True
        verify_components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                             | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                             | PlanningSceneComponents.ROBOT_STATE)
        attached_scene = get_scene(verify_components)
        summary["attachment"] = attachment_status(attached_scene, selected_id, "ee_palm")
        if not summary["attachment"]["valid"]:
            raise RuntimeError("attachment verification failed")

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = target["planning_frame"]
        fk_request.fk_link_names = ["tool0"]
        fk_request.robot_state = attached_scene.robot_state
        fk = call(fk_client, fk_request)
        if fk.error_code.val != MoveItErrorCodes.SUCCESS or len(fk.pose_stamped) != 1:
            raise RuntimeError(f"retreat FK failed: {fk.error_code.val}")
        current_tool_pose = copy.deepcopy(fk.pose_stamped[0])
        lift_pose = translated_pose(current_tool_pose, dz=args.retreat_distance)
        lift_plan, lift_points = plan_pose(attached_scene, lift_pose, "attached lift")
        summary.update({"attached_lift_moveit_error_code": lift_plan.error_code.val,
                        "attached_lift_trajectory_point_count": lift_points,
                        "attached_lift_execution_error_code": execute(
                            lift_plan.trajectory, "attached lift")})
        lifted_scene = get_scene(verify_components)

        place_target = summary["place_target"]["pose_xyz"]
        place_dx = place_target[0] - target["target_pose"][0]
        place_dy = place_target[1] - target["target_pose"][1]
        place_dz = place_target[2] - target["target_pose"][2]
        above_place_pose = translated_pose(
            current_tool_pose, place_dx, place_dy,
            place_dz + args.retreat_distance)
        transfer_plan, transfer_points = plan_pose(
            lifted_scene, above_place_pose, "above-place transfer")
        summary.update({"transfer_moveit_error_code": transfer_plan.error_code.val,
                        "transfer_trajectory_point_count": transfer_points,
                        "transfer_execution_error_code": execute(
                            transfer_plan.trajectory, "above-place transfer")})
        above_place_scene = get_scene(verify_components)

        place_pose = translated_pose(current_tool_pose, place_dx, place_dy, place_dz)
        place_plan, place_points = plan_pose(above_place_scene, place_pose, "place")
        summary.update({"place_moveit_error_code": place_plan.error_code.val,
                        "place_trajectory_point_count": place_points,
                        "place_execution_error_code": execute(place_plan.trajectory, "place")})

        place_diff = place_detachment_diff(original, "ee_palm", place_target)
        if not call(apply_client, ApplyPlanningScene.Request(scene=place_diff)).success:
            raise RuntimeError("PlanningScene rejected place detachment")
        attached = False
        placed_scene = get_scene(verify_components)
        placed_status = attachment_status(placed_scene, selected_id, "ee_palm")
        placed_world = [obj for obj in placed_scene.world.collision_objects
                        if obj.id == selected_id]
        summary["detachment"] = {
            "attached_object_present": placed_status["attached_object_present"],
            "world_object_count": len(placed_world),
            "world_object_present": len(placed_world) == 1,
        }
        if placed_status["attached_object_present"] or len(placed_world) != 1:
            raise RuntimeError("place detachment verification failed")
        set_ownership("")
        summary["ownership_released"] = True

        final_fk_request = GetPositionFK.Request()
        final_fk_request.header.frame_id = target["planning_frame"]
        final_fk_request.fk_link_names = ["tool0"]
        final_fk_request.robot_state = placed_scene.robot_state
        final_fk = call(fk_client, final_fk_request)
        retreat_pose = copy.deepcopy(final_fk.pose_stamped[0])
        retreat_pose.pose.position.z += args.retreat_distance
        retreat_ik_request = GetPositionIK.Request()
        retreat_ik_request.ik_request.group_name = "manipulator"
        retreat_ik_request.ik_request.ik_link_name = "tool0"
        retreat_ik_request.ik_request.pose_stamped = retreat_pose
        retreat_ik_request.ik_request.robot_state = placed_scene.robot_state
        retreat_ik_request.ik_request.avoid_collisions = True
        retreat_ik_request.ik_request.timeout.sec = 3
        retreat_ik = call(ik_client, retreat_ik_request, timeout=6.0)
        if retreat_ik.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"retreat IK failed: {retreat_ik.error_code.val}")
        retreat_plan = plan_to_joint_state(placed_scene.robot_state,
                                           retreat_ik.solution.joint_state)
        retreat_points = len(retreat_plan.trajectory.joint_trajectory.points)
        if retreat_plan.error_code.val != MoveItErrorCodes.SUCCESS or not retreat_points:
            raise RuntimeError(f"retreat planning failed: {retreat_plan.error_code.val}")
        summary.update({"retreat_moveit_error_code": retreat_plan.error_code.val,
                        "retreat_trajectory_point_count": retreat_points,
                        "retreat_execution_error_code": execute(retreat_plan.trajectory, "retreat")})
        final_scene = get_scene(verify_components)
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
        summary["result"] = "PASS"
    except Exception as exc:
        summary["failure"] = str(exc)
        if attached and selected_id:
            # Fail closed by removing a stale attachment. The perceived bridge may
            # re-add a still-live world object on its next update.
            cleanup = detachment_cleanup_diff(selected_id, "ee_palm")
            try:
                summary["cleanup_performed"] = bool(call(
                    apply_client, ApplyPlanningScene.Request(scene=cleanup)).success)
            except Exception as cleanup_exc:
                summary["cleanup_failure"] = str(cleanup_exc)
    finally:
        if summary.get("ownership_claimed_id") and not summary.get("ownership_released"):
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
