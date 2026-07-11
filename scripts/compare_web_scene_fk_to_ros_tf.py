#!/usr/bin/env python3
"""Compare Workcell Studio web FK poses with ROS/RViz-style URDF TF.

This script expands the same scene URDF xacro used by RViz, recomputes the
robot_state_publisher forward kinematics from the expanded URDF joint tree using
Workcell Studio's preview joint values, and compares those poses with the web
scene export.  On success it stamps generated URDF web rows as verified so the
browser can render their baked visual world poses directly.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import extract_scene_urdf_visual_mesh_index as fk  # noqa: E402

IMPORTANT_LINKS = [
    "base_link",
    "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link",
    "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "gripper_base_link",
    "gripper_finger1_knuckle_link", "gripper_finger2_knuckle_link",
    "gripper_finger1_finger_link", "gripper_finger2_finger_link",
    "gripper_finger1_inner_knuckle_link", "gripper_finger2_inner_knuckle_link",
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link",
]
DEFAULT_TOLERANCE = 1e-4


def _scene_path(value: str) -> Path:
    p = Path(value)
    if p.is_absolute():
        return p
    return ROOT / p if p.parts[:1] == ("scenes",) else ROOT / "scenes" / p


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _pose_tuple(pose: Mapping[str, Any] | None) -> tuple[list[float], list[float]]:
    pose = pose if isinstance(pose, Mapping) else {}
    xyz = pose.get("xyz") if isinstance(pose.get("xyz"), list) else []
    rpy = pose.get("rpy") if isinstance(pose.get("rpy"), list) else []
    return ([float(xyz[i]) if i < len(xyz) else 0.0 for i in range(3)], [float(rpy[i]) if i < len(rpy) else 0.0 for i in range(3)])


def _angle_delta(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _delta(a: Mapping[str, Any] | None, b: Mapping[str, Any] | None) -> dict[str, Any]:
    ax, ar = _pose_tuple(a); bx, br = _pose_tuple(b)
    dxyz = [ax[i] - bx[i] for i in range(3)]
    drpy = [_angle_delta(ar[i], br[i]) for i in range(3)]
    return {"xyz": dxyz, "rpy": drpy, "xyz_norm": math.sqrt(sum(v * v for v in dxyz)), "rpy_max": max(drpy)}


def _resolve_urdf(scene_dir: Path, workspace_root: str = "") -> tuple[str, dict[str, Any]]:
    manifest = fk.read_yaml(scene_dir / "scene_manifest.yaml") or {}
    cli = {"use_fake_hardware": "true", "robot_prefix": "", "tool_prefix": ""}
    req = fk._extract_scene_launch_xacro_request(scene_dir, cli)
    if req:
        urdf_path = scene_dir / req.get("rel_path", "urdf/scene.urdf.xacro")
        xargs = dict(req.get("mappings") or {})
    else:
        urdf_path = scene_dir / (((manifest.get("files") or {}).get("urdf_xacro")) or "urdf/scene.urdf.xacro")
        xargs = dict(cli)
    result = fk.expand_xacro(urdf_path, scene_dir, xargs, workspace_root=(workspace_root or None))
    xml_text, _out, err, cmd = result[:4]
    if not xml_text:
        raise RuntimeError(f"xacro expansion failed for {urdf_path}: {err}")
    xml_text, _ = fk.inject_missing_robotiq_85_visuals(xml_text)
    return xml_text, {"urdf_path": fk._repo_relative_path(urdf_path), "xacro_command": fk._portable_source_metadata(cmd)}


def _computed_items(scene_dir: Path, xml_text: str, workspace_root: str = "") -> list[dict[str, Any]]:
    packages = sorted(fk.extract_referenced_package_names(xml_text))
    package_map, _diag = fk.discover_package_map(scene_dir, workspace_root=(workspace_root or None), package_names=packages)
    items = fk.extract_from_urdf(xml_text, package_map)
    # Real xacro in minimal environments can expand the UR joint tree while the
    # upstream UR macro omits visual rows.  Add the same capability-based UR5 mesh
    # visuals so comparison still uses the expanded joint tree poses.
    fk.append_static_ur5_mesh_visuals(items, package_map)
    return [i for i in items if isinstance(i, dict)]


def _web_rows(payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for section in ("robots", "tools", "frames"):
        for item in payload.get(section) or []:
            if isinstance(item, dict) and (item.get("urdf_fk_source") == "expanded_urdf_joint_tree" or str(item.get("link") or "") in IMPORTANT_LINKS):
                rows.append(item)
    return rows



def _matrix_from_pose(pose: Mapping[str, Any] | None) -> list[float] | None:
    if pose is None:
        return None
    xyz, rpy = _pose_tuple(pose)
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    # Rz * Ry * Rx, returned as THREE/ROS column-major Matrix4 elements.
    r00 = cy * cp; r01 = cy * sp * sr - sy * cr; r02 = cy * sp * cr + sy * sr
    r10 = sy * cp; r11 = sy * sp * sr + cy * cr; r12 = sy * sp * cr - cy * sr
    r20 = -sp;     r21 = cp * sr;                  r22 = cp * cr
    return [r00, r10, r20, 0.0, r01, r11, r21, 0.0, r02, r12, r22, 0.0, xyz[0], xyz[1], xyz[2], 1.0]


def _matrix_translation(m: Sequence[float] | None) -> list[float]:
    return [float(m[12]), float(m[13]), float(m[14])] if m and len(m) >= 15 else [0.0, 0.0, 0.0]


def _matrix_rotation_angle(a: Sequence[float] | None, b: Sequence[float] | None) -> float:
    if not a or not b or len(a) < 11 or len(b) < 11:
        return math.inf
    # Column-major rotation matrices; compute trace(Ra^T Rb).
    trace = 0.0
    for row in range(3):
        for col in range(3):
            trace += float(a[col * 4 + row]) * float(b[col * 4 + row])
    value = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return abs(math.acos(value))


def _matrix_delta(a: Sequence[float] | None, b: Sequence[float] | None) -> dict[str, Any]:
    at = _matrix_translation(a); bt = _matrix_translation(b)
    dxyz = [at[i] - bt[i] for i in range(3)]
    return {"xyz": dxyz, "xyz_norm": math.sqrt(sum(v * v for v in dxyz)), "angle": _matrix_rotation_angle(a, b)}


def _browser_status(path_value: str | None) -> Mapping[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value)
    if not path.is_absolute():
        path = ROOT / path
    data = _load_json(path)
    status = data.get("browser", {}).get("status") if isinstance(data.get("browser"), Mapping) else data.get("status")
    return status if isinstance(status, Mapping) else data


def _browser_link_matrices(status: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if not status:
        return {}
    value = status.get("robot_link_world_matrices") or status.get("robotLinkWorldMatrices") or {}
    return value if isinstance(value, Mapping) else {}


def _browser_visual_matrices(status: Mapping[str, Any] | None) -> list[Mapping[str, Any]]:
    if not status:
        return []
    value = status.get("robot_visual_wrapper_world_matrices") or status.get("robotVisualWrapperWorldMatrices") or []
    return [v for v in value if isinstance(v, Mapping)] if isinstance(value, list) else []

def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--web-scene", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE)
    ap.add_argument("--workspace-root", default=os.environ.get("WORKSPACE_ROOT", ""))
    ap.add_argument("--browser-status", help="Visual acceptance report/status JSON containing actual browser Object3D matrixWorld diagnostics.")
    args = ap.parse_args(argv)

    scene_dir = _scene_path(args.scene)
    web_path = Path(args.web_scene)
    if not web_path.is_absolute(): web_path = ROOT / web_path
    out_path = Path(args.output)
    if not out_path.is_absolute(): out_path = ROOT / out_path
    web = _load_json(web_path)
    browser_status = _browser_status(args.browser_status)
    browser_links = _browser_link_matrices(browser_status)
    browser_visuals = _browser_visual_matrices(browser_status)
    xml_text, source = _resolve_urdf(scene_dir, args.workspace_root)
    computed = _computed_items(scene_dir, xml_text, args.workspace_root)

    computed_by_link: dict[str, dict[str, Any]] = {}
    for item in computed:
        link = str(item.get("link") or "")
        if link and link not in computed_by_link:
            computed_by_link[link] = item
    web_by_link: dict[str, dict[str, Any]] = {}
    for item in _web_rows(web):
        link = str(item.get("link") or "")
        if link and link not in web_by_link:
            web_by_link[link] = item

    rows = []
    ok = True
    for link in IMPORTANT_LINKS:
        ros_item = computed_by_link.get(link)
        web_item = web_by_link.get(link)
        if link == "base_link" and not ros_item:
            ros_link = {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
            ros_visual = ros_link
        else:
            ros_link = (ros_item.get("urdf_fk_link_world_pose") or ros_item.get("link_world_pose") or ros_item.get("frame_world_pose") or ros_item.get("final_transform")) if ros_item else None
            ros_visual = (ros_item.get("urdf_fk_visual_world_pose") or ros_item.get("baked_world_visual_pose") or ros_item.get("expected_visual_pose") or ros_item.get("final_transform")) if ros_item else None
        web_link = (web_item or {}).get("urdf_fk_link_world_pose") or (web_item or {}).get("link_world_pose") or (web_item or {}).get("frame_world_pose") or (web_item or {}).get("final_transform")
        web_visual = (web_item or {}).get("urdf_fk_visual_world_pose") or (web_item or {}).get("baked_world_visual_pose") or (web_item or {}).get("expected_visual_pose") or (web_item or {}).get("final_transform")
        actual_browser_link = browser_links.get(link) if isinstance(browser_links.get(link), Mapping) else None
        browser_link_matrix = actual_browser_link.get("matrix_world") if actual_browser_link else None
        expected_link_matrix = _matrix_from_pose(ros_link)
        d_link = _matrix_delta(expected_link_matrix, browser_link_matrix) if browser_status else _delta(ros_link, web_link)
        visual_candidates = [v for v in browser_visuals if str(v.get("link_name") or v.get("linkName") or "") == link]
        browser_visual_matrix = visual_candidates[0].get("matrix_world") if visual_candidates else None
        expected_visual_matrix = _matrix_from_pose(ros_visual)
        d_visual = _matrix_delta(expected_visual_matrix, browser_visual_matrix) if browser_status and browser_visual_matrix else _delta(ros_visual, web_visual)
        link_ok = d_link["xyz_norm"] <= args.tolerance and (d_link.get("angle", d_link.get("rpy_max", math.inf)) <= args.tolerance)
        visual_ok = d_visual["xyz_norm"] <= args.tolerance and (d_visual.get("angle", d_visual.get("rpy_max", math.inf)) <= args.tolerance)
        present_ok = bool((ros_item or link == "base_link") and ((actual_browser_link if browser_status else web_item)))
        passed = bool(present_ok and link_ok and visual_ok)
        ok = ok and passed
        rows.append({
            "link_name": link,
            "ros_tf_xyz_rpy": ros_link,
            "ros_tf_visual_xyz_rpy": ros_visual,
            "exported_web_fk_xyz_rpy": web_link,
            "browser_object3d_matrix_world": browser_link_matrix,
            "exported_web_fk_visual_xyz_rpy": web_visual,
            "browser_visual_wrapper_matrix_world": browser_visual_matrix,
            "delta_link": d_link,
            "delta_visual": d_visual,
            "pass": passed,
        })
        link_angle = d_link.get('angle', d_link.get('rpy_max', math.inf))
        visual_angle = d_visual.get('angle', d_visual.get('rpy_max', math.inf))
        print(f"{link}: xyz_delta={d_link['xyz_norm']:.6g} angle_delta={link_angle:.6g} visual_xyz_delta={d_visual['xyz_norm']:.6g} visual_angle_delta={visual_angle:.6g} {'PASS' if passed else 'FAIL'}")

    report = {"scene": scene_dir.name, "source": source, "tolerance": args.tolerance, "compared_browser_object3d_matrices": bool(browser_status), "passed": ok, "links": rows}
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if ok:
        for item in _web_rows(web):
            item["urdf_fk_verified_against_ros_tf"] = True
            item.setdefault("urdf_fk_source", "expanded_urdf_joint_tree")
            item.setdefault("urdf_fk_link_world_pose", item.get("link_world_pose") or item.get("frame_world_pose") or item.get("final_transform"))
            item.setdefault("urdf_fk_visual_world_pose", item.get("baked_world_visual_pose") or item.get("expected_visual_pose") or item.get("final_transform"))
            item["robot_transform_source"] = "ros_tf_verified_urdf_fk"
            item["robot_render_mode"] = "verified_urdf_fk_visual_world_pose"
            item["workcell_web_render_pose_mode"] = "verified_urdf_fk_visual_world_pose"
        web.setdefault("diagnostics", {})["fk_vs_ros_tf_report"] = fk._repo_relative_path(out_path)
        web_path.write_text(json.dumps(web, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
