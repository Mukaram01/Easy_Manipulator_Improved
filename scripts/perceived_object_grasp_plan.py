#!/usr/bin/env python3
"""Plan, but never execute, a grasp derived from a perceived PlanningScene box."""

import argparse
import copy
import json
import math
import time


ARM_JOINT_SUFFIXES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


def _finite(values):
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values)


def select_perceived_box(objects):
    """Select the lowest stable ID, rejecting incomplete or fabricated geometry."""
    valid = []
    for obj in objects:
        object_id = str(obj.get("id", ""))
        dimensions = obj.get("dimensions")
        pose = obj.get("pose")
        if (not object_id or obj.get("shape") != "BOX"
                or not isinstance(dimensions, (list, tuple)) or len(dimensions) != 3
                or not _finite(dimensions) or any(value <= 0.0 for value in dimensions)
                or not isinstance(pose, (list, tuple)) or len(pose) != 7
                or not _finite(pose)):
            continue
        valid.append(obj)
    if not valid:
        raise ValueError("no perceived BOX has finite positive geometry and pose")
    return min(valid, key=lambda obj: (not str(obj["id"]).isdigit(),
                                       int(obj["id"]) if str(obj["id"]).isdigit() else str(obj["id"])))


def build_grasp_target(selected):
    """Map PlanningScene truth to the existing GraspTarget-shaped fields."""
    return {
        "perceived_object_id": str(selected["id"]),
        "target_type": "BOX",
        "planning_frame": selected["frame_id"],
        "target_pose": list(selected["pose"]),
        "target_dimensions": list(selected["dimensions"]),
    }


def generate_box_grasp_candidates(target, clearance=0.12):
    """Generate deterministic top approaches; positions always derive from the box."""
    x, y, z, _, _, _, _ = target["target_pose"]
    height = target["target_dimensions"][2]
    clearances = [clearance] if isinstance(clearance, (int, float)) else list(clearance)
    # Tool Z down, with four deterministic rotations about world Z.
    candidates = []
    for offset in clearances:
        for yaw in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            # qz(yaw)*qx(pi), followed by qz(yaw)*qy(pi). Both point tool Z down.
            candidates.append([x, y, z + 0.5 * height + offset,
                               math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0])
            candidates.append([x, y, z + 0.5 * height + offset,
                               -math.sin(yaw / 2.0), math.cos(yaw / 2.0), 0.0, 0.0])
    return candidates


class ExecutionGuard:
    def __init__(self):
        self.execution_attempted = False

    def forbid_execution(self):
        self.execution_attempted = True
        raise RuntimeError("trajectory execution is forbidden in P8-E1")


def _collision_object_dict(obj):
    if len(obj.primitives) != 1 or len(obj.primitive_poses) != 1 or obj.primitives[0].type != 1:
        return None
    primitive = obj.primitives[0]
    # MoveIt may canonicalize a CollisionObject by moving the world transform to
    # object.pose and leaving the primitive pose at identity. Compose both.
    base = obj.pose
    local = obj.primitive_poses[0]
    bx, by, bz, bw = (base.orientation.x, base.orientation.y,
                      base.orientation.z, base.orientation.w)
    lx, ly, lz, lw = (local.orientation.x, local.orientation.y,
                      local.orientation.z, local.orientation.w)
    if bx * bx + by * by + bz * bz + bw * bw < 1e-12:
        bx, by, bz, bw = 0.0, 0.0, 0.0, 1.0
    if lx * lx + ly * ly + lz * lz + lw * lw < 1e-12:
        lx, ly, lz, lw = 0.0, 0.0, 0.0, 1.0
    vx, vy, vz = local.position.x, local.position.y, local.position.z
    # Rotate the local translation by the base quaternion.
    tx = 2.0 * (by * vz - bz * vy)
    ty = 2.0 * (bz * vx - bx * vz)
    tz = 2.0 * (bx * vy - by * vx)
    rx = vx + bw * tx + (by * tz - bz * ty)
    ry = vy + bw * ty + (bz * tx - bx * tz)
    rz = vz + bw * tz + (bx * ty - by * tx)
    qx = bw * lx + bx * lw + by * lz - bz * ly
    qy = bw * ly - bx * lz + by * lw + bz * lx
    qz = bw * lz + bx * ly - by * lx + bz * lw
    qw = bw * lw - bx * lx - by * ly - bz * lz
    return {
        "id": obj.id,
        "shape": "BOX",
        "frame_id": obj.header.frame_id,
        "dimensions": list(primitive.dimensions),
        "pose": [base.position.x + rx, base.position.y + ry, base.position.z + rz,
                 qx, qy, qz, qw],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary-output", required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
    from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionIK

    rclpy.init()
    node = rclpy.create_node("perceived_object_grasp_plan")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    guard = ExecutionGuard()
    summary = {"result": "FAIL", "execution_attempted": False}

    def call(client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service timed out: {client.srv_name}")
        return future.result()

    try:
        deadline = time.monotonic() + args.timeout
        selected = None
        scene_response = None
        while time.monotonic() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                              | PlanningSceneComponents.ROBOT_STATE)
            scene_response = call(scene_client, request)
            objects = [item for item in
                       (_collision_object_dict(obj) for obj in scene_response.scene.world.collision_objects)
                       if item and item["id"].isdigit()]
            try:
                selected = select_perceived_box(objects)
                break
            except ValueError:
                time.sleep(0.1)
        if selected is None:
            raise RuntimeError("timed out waiting for a valid perceived PlanningScene BOX")

        target = build_grasp_target(selected)
        candidates = generate_box_grasp_candidates(target, clearance=(0.32, 0.42))
        ids = [obj.id for obj in scene_response.scene.world.collision_objects if obj.id.isdigit()]
        summary.update({
            "selected_object_id": target["perceived_object_id"],
            "object_pose": target["target_pose"],
            "object_dimensions": target["target_dimensions"],
            "planning_frame": target["planning_frame"],
            "planning_group": "manipulator",
            "end_effector": "tool0",
            "grasp_candidates_generated": len(candidates),
            "planning_scene_selected_id_count": ids.count(target["perceived_object_id"]),
            "duplicate_ids": sorted({item for item in ids if ids.count(item) > 1}),
            "attempted_candidate_count": 0,
            "candidate_results": [],
        })

        for index, values in enumerate(candidates):
            summary["attempted_candidate_count"] += 1
            pose = PoseStamped()
            pose.header.frame_id = target["planning_frame"]
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
             pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = values
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = "manipulator"
            # The manipulator kinematics chain ends at tool0; ee_palm is a fixed
            # downstream grasp frame and is not accepted by the UR IK plugin.
            ik_request.ik_request.ik_link_name = "tool0"
            ik_request.ik_request.pose_stamped = pose
            ik_request.ik_request.robot_state = scene_response.scene.robot_state
            # IK supplies joint-space goal constraints only. The authoritative
            # /plan_kinematic_path call below performs collision checking.
            ik_request.ik_request.avoid_collisions = False
            ik_request.ik_request.timeout.sec = 2
            ik = call(ik_client, ik_request, timeout=5.0)
            if ik.error_code.val != MoveItErrorCodes.SUCCESS:
                summary["candidate_results"].append(
                    {"index": index, "ik_error_code": ik.error_code.val})
                continue

            goal = Constraints()
            for name, position in zip(ik.solution.joint_state.name, ik.solution.joint_state.position):
                if any(name.endswith(suffix) for suffix in ARM_JOINT_SUFFIXES):
                    constraint = JointConstraint()
                    constraint.joint_name = name
                    constraint.position = position
                    constraint.tolerance_above = 0.001
                    constraint.tolerance_below = 0.001
                    constraint.weight = 1.0
                    goal.joint_constraints.append(constraint)
            plan_request = GetMotionPlan.Request()
            motion = plan_request.motion_plan_request
            motion.group_name = "manipulator"
            motion.start_state = copy.deepcopy(scene_response.scene.robot_state)
            motion.start_state.is_diff = True
            motion.goal_constraints = [goal]
            motion.num_planning_attempts = 3
            motion.allowed_planning_time = 5.0
            motion.max_velocity_scaling_factor = 0.2
            motion.max_acceleration_scaling_factor = 0.2
            plan = call(plan_client, plan_request, timeout=10.0).motion_plan_response
            points = len(plan.trajectory.joint_trajectory.points)
            summary["moveit_error_code"] = plan.error_code.val
            summary["candidate_results"].append(
                {"index": index, "ik_error_code": ik.error_code.val,
                 "moveit_error_code": plan.error_code.val, "trajectory_point_count": points})
            if plan.error_code.val == MoveItErrorCodes.SUCCESS and points > 0:
                summary.update({"result": "PASS", "successful_candidate_index": index,
                                "trajectory_point_count": points})
                break
        if summary["result"] != "PASS":
            raise RuntimeError("no grasp candidate produced a non-empty MoveIt plan")
        if summary["planning_scene_selected_id_count"] != 1 or summary["duplicate_ids"]:
            raise RuntimeError("selected perceived ID was not unique during planning")
    except Exception as exc:
        summary["failure"] = str(exc)
    finally:
        summary["execution_attempted"] = guard.execution_attempted
        with open(args.summary_output, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(summary, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary["result"] == "PASS" and not summary["execution_attempted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
