#!/usr/bin/env python3
"""Stable UR5/Robotiq preview row definitions for Scene3D visual-index repair."""
from __future__ import annotations

from typing import Any

REQUIRED_UR5_VISUALS: tuple[dict[str, Any], ...] = (
    {"link": "base_link_inertia", "mesh": "base.dae", "xyz": [0.0, 0.0, 0.08], "rpy": [0.0, 0.0, 0.0], "parent": "world", "chain": ["world", "base_link_inertia"], "size": [0.26, 0.26, 0.16]},
    {"link": "shoulder_link", "mesh": "shoulder.dae", "xyz": [0.0, 0.0, 0.22], "rpy": [0.0, 0.0, 0.0], "parent": "base_link_inertia", "chain": ["world", "base_link_inertia", "shoulder_link"], "size": [0.18, 0.18, 0.26]},
    {"link": "upper_arm_link", "mesh": "upperarm.dae", "xyz": [0.27, 0.0, 0.58], "rpy": [0.0, 0.35, 0.0], "parent": "shoulder_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link"], "size": [0.54, 0.11, 0.13]},
    {"link": "forearm_link", "mesh": "forearm.dae", "xyz": [0.58, 0.0, 0.43], "rpy": [0.0, -0.45, 0.0], "parent": "upper_arm_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link"], "size": [0.48, 0.10, 0.12]},
    {"link": "wrist_1_link", "mesh": "wrist1.dae", "xyz": [0.82, 0.0, 0.31], "rpy": [0.0, 0.0, 0.0], "parent": "forearm_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link"], "size": [0.12, 0.10, 0.16]},
    {"link": "wrist_2_link", "mesh": "wrist2.dae", "xyz": [0.92, 0.0, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "wrist_1_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link"], "size": [0.12, 0.10, 0.12]},
    {"link": "wrist_3_link", "mesh": "wrist3.dae", "xyz": [1.00, 0.0, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "wrist_2_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"], "size": [0.10, 0.10, 0.10]},
)

# Match the link names emitted by the bundled robotiq_85_gripper xacro.  Earlier
# stable preview rows used synthetic robotiq_85_* link names, which could make
# the Builder canvas show a detached/fake end-effector even when the generated
# URDF and RViz use gripper_base_link / gripper_finger*_finger_tip_link.
STABLE_UR5_2F_PREVIEW_VISUALS: tuple[dict[str, Any], ...] = (
    {"link": "tool0", "mesh": "", "xyz": [1.08, 0.0, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "wrist_3_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"], "size": [0.10, 0.10, 0.08], "category": "end_effector", "role": "end_effector", "preview_model": "robotiq_85"},
    {"link": "gripper_base_link", "mesh": "robotiq_85_base_link.dae", "mesh_prefix": "package://robotiq_85_description/meshes/visual/", "xyz": [1.17, 0.0, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "tool0", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link"], "size": [0.12, 0.08, 0.10], "category": "end_effector", "role": "end_effector", "preview_model": "robotiq_85"},
    {"link": "gripper_finger1_finger_tip_link", "mesh": "robotiq_85_finger_tip_link.dae", "mesh_prefix": "package://robotiq_85_description/meshes/visual/", "xyz": [1.26, 0.045, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "gripper_base_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link", "gripper_finger1_finger_tip_link"], "size": [0.12, 0.025, 0.08], "category": "end_effector", "role": "end_effector", "preview_model": "robotiq_85"},
    {"link": "gripper_finger2_finger_tip_link", "mesh": "robotiq_85_finger_tip_link.dae", "mesh_prefix": "package://robotiq_85_description/meshes/visual/", "xyz": [1.26, -0.045, 0.28], "rpy": [0.0, 0.0, 0.0], "parent": "gripper_base_link", "chain": ["world", "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link", "gripper_finger2_finger_tip_link"], "size": [0.12, 0.025, 0.08], "category": "end_effector", "role": "end_effector", "preview_model": "robotiq_85"},
)

REQUIRED_UR5_LINKS = tuple(str(spec["link"]) for spec in REQUIRED_UR5_VISUALS)
STABLE_UR5_2F_LINKS = tuple(str(spec["link"]) for spec in STABLE_UR5_2F_PREVIEW_VISUALS)


def _pose(xyz: list[float], rpy: list[float]) -> dict[str, list[float]]:
    return {"xyz": [float(v) for v in xyz], "rpy": [float(v) for v in rpy]}


def make_ur5_preview_item(spec: dict[str, Any], index: int) -> dict[str, Any]:
    link = str(spec["link"])
    mesh_name = str(spec.get("mesh") or "")
    mesh_prefix = str(spec.get("mesh_prefix") or "package://ur_description/meshes/ur5/visual/")
    reference_uri = str(spec.get("reference_mesh_uri") or (f"{mesh_prefix}{mesh_name}" if mesh_name else ""))
    pose = _pose(list(spec["xyz"]), list(spec["rpy"]))
    category = str(spec.get("category") or "robot")
    role = str(spec.get("role") or "robot")
    preview_model = str(spec.get("preview_model") or "ur5")
    metadata_tags = (
        f"source=urdf_flattened;stable_scene3d_ur5_builder_preview;"
        f"rviz_launch_authoritative;category={category};robot_model=ur5;preview_model={preview_model}"
    )
    link_chain = list(spec.get("chain") or ["world", link])
    parent = str(spec.get("parent") or "")
    return {
        "id": f"generated_urdf::{link}::stable_preview::{index}",
        "item_id": f"generated_urdf::{link}",
        "source": "urdf_flattened",
        "source_layer": "locked_generated_urdf_visual",
        "active_visual_source": "primitive_fallback",
        "category": category,
        "role": role,
        "robot_model": "ur5",
        "preview_model": preview_model,
        "preview_locked": True,
        "editable": False,
        "generated_urdf_visual": True,
        "locked_generated_urdf_visual": True,
        "link": link,
        "link_name": link,
        "object_name": link,
        "canonical_link_name": link,
        "render_identity": link,
        "final_render_identity": link,
        "final_render_link": link,
        "final_draw_link": link,
        "visual": f"stable_preview_{index}",
        "visual_name": f"stable_preview_{index}",
        "visual_index": index,
        "source_row_index": index,
        "parent_link": parent,
        "immediate_parent_link": parent,
        "root_link": "world",
        "link_chain": link_chain,
        "joint_parent_link": parent,
        "pose": pose,
        "chain_pose": pose,
        "world_pose": pose,
        "link_world_pose": pose,
        "expected_visual_pose": pose,
        "baked_world_visual_pose": pose,
        "baked_world_visual_transform_source": "stable_scene3d_ur5_builder_preview",
        "visual_origin_applied_to_pose": True,
        "visual_origin": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        "geometry_type": "box",
        "primitive_geometry_type": "box",
        "size": [float(v) for v in spec["size"]],
        "has_mesh_metadata": False,
        "mesh_available": False,
        "reference_mesh_uri": reference_uri,
        "mesh_source": "",
        "mesh_uri": "",
        "package_uri": "",
        "source_path": "",
        "mesh_path": "",
        "mesh_file_name": mesh_name,
        "resolved": True,
        "mesh_scale": [1.0, 1.0, 1.0],
        "scale": [1.0, 1.0, 1.0],
        "render_expected": True,
        "render_skip_reason": "",
        "warning": "",
        "material": {"name": "ur5_stable_builder_preview", "color": [0.85, 0.88, 0.92, 1.0]},
        "color": [0.85, 0.88, 0.92, 1.0],
        "link_transform_status": "stable_scene3d_ur5_builder_preview",
        "transform_status": "stable_scene3d_ur5_builder_preview",
        "transform_chain": link_chain,
        "visual_index_link": link,
        "visual_index_link_name": link,
        "visual_index_object_name": link,
        "visual_index_visual": f"stable_preview_{index}",
        "visual_index_visual_name": f"stable_preview_{index}",
        "visual_index_value": index,
        "visual_index_parent_link": parent,
        "visual_index_link_chain": link_chain,
        "visual_index_mesh_uri": reference_uri,
        "visual_index_package_uri": reference_uri,
        "visual_index_source": "stable_scene3d_ur5_builder_preview",
        "metadata_tags": metadata_tags,
    }
