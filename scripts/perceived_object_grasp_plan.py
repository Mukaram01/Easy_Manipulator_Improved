#!/usr/bin/env python3
"""Plan, but never execute, a grasp derived from a perceived PlanningScene box."""

import argparse
import copy
import json
import math
from pathlib import Path
import time


ARM_JOINT_SUFFIXES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)


def _finite(values):
    return all(isinstance(value, (int, float)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values)


def select_perceived_box(objects):
    """Select one live EPD box, rejecting authored/fabricated geometry."""
    valid = []
    for obj in objects:
        object_id = str(obj.get("id", ""))
        # The bridge owns the ``epd::`` namespace. Keep numeric IDs as a
        # compatibility path for older replay fixtures, but never select a
        # workcell-authored collision object as the perceived workpiece.
        if not (object_id.startswith("epd::") or object_id.isdigit()):
            continue
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
    return min(valid, key=lambda obj: str(obj["id"]))


def load_grasp_contract(scene_package):
    """Resolve a source directory or installed package; reject missing metadata."""
    package = Path(scene_package)
    if not package.is_dir():
        from ament_index_python.packages import get_package_share_directory
        package = Path(get_package_share_directory(str(scene_package)))
    source = package / "cell_definition.yaml"
    import yaml
    cell = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    grasp = cell.get("task", {}).get("grasp") or cell.get("grasp") or {}
    end_effector = cell.get("end_effector") or {}
    tcp_xyz = end_effector.get("tcp_pose_xyz")
    tcp_rpy = end_effector.get("tcp_pose_rpy")
    for name, values in (("tcp_pose_xyz", tcp_xyz), ("tcp_pose_rpy", tcp_rpy)):
        if not isinstance(values, list) or len(values) != 3 or not _finite(values):
            raise ValueError(f"{source}: missing or invalid end_effector.{name}")
    distances = [grasp.get("approach_distance_m"), grasp.get("retreat_distance_m")]
    if not _finite(distances) or any(v <= 0 for v in distances):
        raise ValueError(f"{source}: approach/retreat distances must be positive")
    links = end_effector.get("allowed_touch_links")
    if not isinstance(links, list) or not links or any(not isinstance(v, str) or not v for v in links):
        raise ValueError(f"{source}: allowed_touch_links must designate contact links")
    return {
        "approach_distance_m": float(distances[0]),
        "retreat_distance_m": float(distances[1]),
        "tcp_offset_z_m": float(tcp_xyz[2]),
        "tcp_pose": list(tcp_xyz) + quaternion_from_rpy(tcp_rpy),
        "allowed_touch_links": list(dict.fromkeys(links)),
        "tool_link": cell["robot"]["tool_link"],
        "grasp_frame": end_effector["grasp_frame"],
        "planning_group": cell["robot"]["planning_group"],
        "home_joint_names": cell["robot"]["joint_names"],
        "home_joint_positions": cell["robot"]["safe_joint_state"],
    }


def quaternion_from_rpy(rpy):
    r, p, y = [v / 2 for v in rpy]
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate_vector(q, xyz):
    return quaternion_product(quaternion_product(q, list(xyz) + [0.0]),
                              [-q[0], -q[1], -q[2], q[3]])[:3]


def compose_pose(parent, local):
    rotated = rotate_vector(parent[3:], local[:3])
    return [a+b for a, b in zip(parent[:3], rotated)] + quaternion_product(parent[3:], local[3:])


def inverse_pose(pose):
    q = [-pose[3], -pose[4], -pose[5], pose[6]]
    return rotate_vector(q, [-v for v in pose[:3]]) + q


def tool_pose_for_grasp(grasp_pose, contract):
    """T_world_tool = T_world_grasp * inverse(T_tool_grasp)."""
    return compose_pose(grasp_pose, inverse_pose(contract["tcp_pose"]))


def build_grasp_target(selected):
    """Map PlanningScene truth to the existing GraspTarget-shaped fields."""
    return {
        "perceived_object_id": str(selected["id"]),
        "target_type": "BOX",
        "planning_frame": selected["frame_id"],
        "target_pose": list(selected["pose"]),
        "target_dimensions": list(selected["dimensions"]),
    }


def oriented_box_extents(target):
    """Return the live box extents projected into its planning frame."""
    _, _, _, qx, qy, qz, qw = target["target_pose"]
    rotation = [
        [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
        [2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)],
        [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx*qx + qy*qy)],
    ]
    return [sum(abs(rotation[axis][index]) * target["target_dimensions"][index]
                 for index in range(3)) for axis in range(3)]


def generate_box_grasp_candidates(target, clearance=0.12):
    """Generate deterministic top approaches; positions always derive from the box."""
    x, y, z, qx, qy, qz, qw = target["target_pose"]
    # EPD dimensions are in the object's local frame.  Use the live
    # orientation to calculate the world-Z extent instead of assuming the
    # third dimension is vertical.
    height = oriented_box_extents(target)[2]
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
    parser.add_argument("--scene-package", default="scenes/ur5_2f_test")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
    from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetPositionIK, GetStateValidity
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    rclpy.init()
    node = rclpy.create_node("perceived_object_grasp_plan")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    ownership_client = node.create_client(
        SetParameters, "/epd_dynamic_planning_scene_bridge/set_parameters")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    guard = ExecutionGuard()
    summary = {"result": "FAIL", "execution_attempted": False}
    ownership_claimed = False

    def call(client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service timed out: {client.srv_name}")
        return future.result()

    def set_ownership(object_id, timeout=3.0):
        if not ownership_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("perception bridge ownership service unavailable")
        parameter = Parameter()
        parameter.name = "owned_object_id"
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=object_id)
        response = call(ownership_client, SetParameters.Request(parameters=[parameter]), timeout=timeout)
        if len(response.results) != 1 or not response.results[0].successful:
            raise RuntimeError("perception bridge rejected planning ownership")

    try:
        deadline = time.monotonic() + args.timeout
        selected = None
        grasp_contract = load_grasp_contract(args.scene_package)
        scene_response = None
        while time.monotonic() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                              | PlanningSceneComponents.ROBOT_STATE)
            scene_response = call(scene_client, request)
            objects = [item for item in
                       (_collision_object_dict(obj) for obj in scene_response.scene.world.collision_objects)
                       if item and (str(item["id"]).startswith("epd::") or str(item["id"]).isdigit())]
            try:
                selected = select_perceived_box(objects)
                break
            except ValueError:
                time.sleep(0.1)
        if selected is None:
            raise RuntimeError("timed out waiting for a valid perceived PlanningScene BOX")

        target = build_grasp_target(selected)
        # Freeze the selected stable ID while MoveIt plans against this exact
        # collision geometry. The bridge still fail-closes stale/lost objects;
        # it simply does not replace an object that the planner owns. EPD's
        # CPU inference can briefly leave a two-second freshness window, so
        # retry the ownership claim until the same ID is live again.
        claim_deadline = min(deadline, time.monotonic() + 10.0)
        while True:
            try:
                set_ownership(target["perceived_object_id"], timeout=2.0)
                ownership_claimed = True
                break
            except RuntimeError:
                if time.monotonic() >= claim_deadline:
                    raise
                time.sleep(0.15)
        # Re-read the scene after the bridge acknowledges ownership so the
        # subsequent IK and collision plans use the protected object version.
        refreshed = GetPlanningScene.Request()
        refreshed.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                           | PlanningSceneComponents.ROBOT_STATE)
        scene_response = call(scene_client, refreshed)
        refreshed_objects = [item for item in
                             (_collision_object_dict(obj) for obj in scene_response.scene.world.collision_objects)
                             if item and item["id"] == target["perceived_object_id"]]
        if len(refreshed_objects) != 1:
            raise RuntimeError("owned perceived object was not unique in the refreshed PlanningScene")
        selected = refreshed_objects[0]
        target = build_grasp_target(selected)
        # Generate grasp-frame approaches, then explicitly convert to the IK
        # tip. A fixed downstream URDF joint does not reinterpret a tool0 goal.
        clearance_schedule = [grasp_contract["approach_distance_m"]]
        grasp_candidates = generate_box_grasp_candidates(target, clearance=clearance_schedule[0])
        candidates = [tool_pose_for_grasp(pose, grasp_contract) for pose in grasp_candidates]
        ids = [obj.id for obj in scene_response.scene.world.collision_objects
               if str(obj.id).startswith("epd::") or str(obj.id).isdigit()]
        summary.update({
            "selected_object_id": target["perceived_object_id"],
            "object_pose": target["target_pose"],
            "object_dimensions": target["target_dimensions"],
            "planning_frame": target["planning_frame"],
            "planning_group": grasp_contract["planning_group"],
            "end_effector": grasp_contract["tool_link"],
            "grasp_frame": grasp_contract["grasp_frame"],
            "grasp_frame_candidates": grasp_candidates,
            "tool_frame_candidates": candidates,
            "approach_distance_m": grasp_contract["approach_distance_m"],
            "clearance_schedule_m": clearance_schedule,
            "retreat_distance_m": grasp_contract["retreat_distance_m"],
            "tcp_offset_z_m": grasp_contract["tcp_offset_z_m"],
            "tcp_transform_applied_to_ik_target": True,
            "grasp_candidates_generated": len(candidates),
            "planning_scene_selected_id_count": ids.count(target["perceived_object_id"]),
            "duplicate_ids": sorted({item for item in ids if ids.count(item) > 1}),
            "attempted_candidate_count": 0,
            "candidate_results": [],
        })

        validity = call(validity_client, GetStateValidity.Request(
            robot_state=scene_response.scene.robot_state, group_name=""))
        summary["initial_state_valid"] = validity.valid
        summary["initial_contacts"] = [
            {"a": c.contact_body_1, "b": c.contact_body_2, "depth_m": c.depth}
            for c in validity.contacts]
        if not validity.valid:
            raise RuntimeError("initial robot state is in collision")

        for index, values in enumerate(candidates):
            summary["attempted_candidate_count"] += 1
            pose = PoseStamped()
            pose.header.frame_id = target["planning_frame"]
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
             pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = values
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = grasp_contract["planning_group"]
            # The manipulator kinematics chain ends at tool0; ee_palm is a fixed
            # downstream grasp frame and is not accepted by the UR IK plugin.
            ik_request.ik_request.ik_link_name = grasp_contract["tool_link"]
            ik_request.ik_request.pose_stamped = pose
            ik_request.ik_request.robot_state = scene_response.scene.robot_state
            # Ask the MoveIt IK service for a collision-free branch before
            # constraining the authoritative motion plan.  A geometric IK
            # seed can select a wrist branch that intersects the fixed camera
            # even when another valid branch reaches the same tool pose.
            # /plan_kinematic_path still validates the complete motion.
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.timeout.sec = 2
            ik = call(ik_client, ik_request, timeout=5.0)
            if ik.error_code.val != MoveItErrorCodes.SUCCESS:
                summary["candidate_results"].append(
                    {"index": index, "clearance_m": clearance_schedule[0],
                     "ik_error_code": ik.error_code.val})
                continue

            goal = Constraints()
            for name, position in zip(ik.solution.joint_state.name, ik.solution.joint_state.position):
                if any(name.endswith(suffix) for suffix in ARM_JOINT_SUFFIXES):
                    constraint = JointConstraint()
                    constraint.joint_name = name
                    constraint.position = position
                    # A milliradian-equivalent joint box is unnecessarily
                    # brittle for OMPL's goal sampler; keep the live IK goal
                    # tight while leaving a small collision-checked region.
                    constraint.tolerance_above = 0.005
                    constraint.tolerance_below = 0.005
                    constraint.weight = 1.0
                    goal.joint_constraints.append(constraint)
            plan_request = GetMotionPlan.Request()
            motion = plan_request.motion_plan_request
            motion.group_name = grasp_contract["planning_group"]
            motion.start_state = copy.deepcopy(scene_response.scene.robot_state)
            motion.start_state.is_diff = True
            motion.goal_constraints = [goal]
            motion.num_planning_attempts = 5
            motion.allowed_planning_time = 8.0
            motion.max_velocity_scaling_factor = 0.2
            motion.max_acceleration_scaling_factor = 0.2
            plan = call(plan_client, plan_request, timeout=14.0).motion_plan_response
            points = len(plan.trajectory.joint_trajectory.points)
            summary["moveit_error_code"] = plan.error_code.val
            summary["candidate_results"].append(
                {"index": index, "clearance_m": clearance_schedule[0],
                 "ik_error_code": ik.error_code.val,
                 "moveit_error_code": plan.error_code.val, "trajectory_point_count": points})
            if plan.error_code.val == MoveItErrorCodes.SUCCESS and points > 0:
                summary.update({"result": "PASS", "successful_candidate_index": index,
                                "successful_candidate_clearance_m": clearance_schedule[0],
                                "trajectory_point_count": points})
                break
        if summary["result"] != "PASS":
            raise RuntimeError("no grasp candidate produced a non-empty MoveIt plan")
        if summary["planning_scene_selected_id_count"] != 1 or summary["duplicate_ids"]:
            raise RuntimeError("selected perceived ID was not unique during planning")
    except Exception as exc:
        summary["failure"] = str(exc)
    finally:
        if ownership_claimed:
            try:
                set_ownership("")
                summary["ownership_released"] = True
            except Exception as ownership_exc:
                summary["ownership_release_failure"] = str(ownership_exc)
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
