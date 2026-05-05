#!/usr/bin/env python3
"""Validate deterministic physical layout relationships for ur5_2f_sorting_test."""

from __future__ import annotations

import ast
import math
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import re


EPS = 1e-6
XACRO_NS = "{http://www.ros.org/wiki/xacro}"


def _safe_eval(expression: str, variables: dict[str, float]) -> float:
    expression = expression.strip()

    def _eval(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        if isinstance(node, ast.BinOp):
            left = _eval(node.left)
            right = _eval(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
            raise ValueError(f"unsupported operator: {ast.dump(node.op)}")
        if isinstance(node, ast.UnaryOp):
            value = _eval(node.operand)
            if isinstance(node.op, ast.UAdd):
                return value
            if isinstance(node.op, ast.USub):
                return -value
            raise ValueError(f"unsupported unary operator: {ast.dump(node.op)}")
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in variables:
            return float(variables[node.id])
        raise ValueError(f"unsupported expression component: {ast.dump(node)}")

    parsed = ast.parse(expression, mode="eval")
    return _eval(parsed)


def _parse_value(raw: str, variables: dict[str, float]) -> float:
    value = raw.strip()
    if value.startswith("${") and value.endswith("}"):
        return _safe_eval(value[2:-1], variables)
    return float(value)




def _split_xyz(expr: str) -> list[str]:
    parts: list[str] = []
    token = []
    depth = 0
    for ch in expr.strip():
        if ch == ' ' and depth == 0:
            if token:
                parts.append(''.join(token))
                token = []
            continue
        token.append(ch)
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth = max(0, depth - 1)
    if token:
        parts.append(''.join(token))
    return parts

def _resolve_scene_xacro() -> Path:
    script_dir = Path(__file__).resolve().parent
    source_candidate = script_dir.parent / "urdf" / "scene.urdf.xacro"
    if source_candidate.exists():
        return source_candidate

    if shutil.which("ros2"):
        try:
            share_dir = subprocess.check_output(
                ["ros2", "pkg", "prefix", "ur5_2f_sorting_test"],
                text=True,
            ).strip()
            installed = Path(share_dir) / "share" / "ur5_2f_sorting_test" / "urdf" / "scene.urdf.xacro"
            if installed.exists():
                return installed
        except subprocess.CalledProcessError:
            pass

    raise FileNotFoundError("unable to locate scene.urdf.xacro in source tree or installed share path")


def _load_xml_root(xacro_path: Path) -> ET.Element:
    # Parse the raw xacro source. The validator needs xacro:property tags;
    # those disappear after xacro expansion.
    return ET.parse(xacro_path).getroot()


def _find_joint_origin(root: ET.Element, joint_name: str, variables: dict[str, float]) -> list[float]:
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            origin = joint.find("origin")
            if origin is None or "xyz" not in origin.attrib:
                raise AssertionError(f"joint '{joint_name}' is missing origin xyz")
            return [_parse_value(x, variables) for x in _split_xyz(origin.attrib["xyz"])]
    raise AssertionError(f"joint '{joint_name}' not found")


def main() -> int:
    xacro_path = _resolve_scene_xacro()
    raw_root = ET.parse(xacro_path).getroot()
    root = _load_xml_root(xacro_path)

    properties: dict[str, float] = {}
    for prop in raw_root.findall(f"{XACRO_NS}property"):
        name = prop.get("name")
        value = prop.get("value")
        if name and value:
            properties[name] = _parse_value(value, properties)

    for required in ("floor_z", "table_height", "table_top_z", "tray_height", "tray_length", "tray_width", "tray_wall_thickness"):
        if required not in properties:
            raise AssertionError(f"missing required property '{required}'")

    table_top_z = properties["table_top_z"]
    expected_top_z = properties["floor_z"] + properties["table_height"]
    if not math.isclose(table_top_z, expected_top_z, abs_tol=EPS):
        raise AssertionError(
            f"table_top_z is not deterministic: {table_top_z} != floor_z + table_height ({expected_top_z})"
        )

    if root.find("link[@name='world']") is None:
        raise AssertionError("world link missing")
    if root.find("link[@name='table_']") is None:
        raise AssertionError("table_ link missing")

    world_to_table_z = _find_joint_origin(root, "world_to_table", properties)[2]
    if not math.isclose(world_to_table_z, table_top_z, abs_tol=EPS):
        raise AssertionError("world_to_table z does not equal table_top_z")

    expected_item_bottoms = {
        "item_red": table_top_z,
        "item_blue": table_top_z,
        "item_green": table_top_z,
    }
    item_joints = {
        "item_red": "table_to_item_red",
        "item_blue": "table_to_item_blue",
        "item_green": "table_to_item_green",
    }
    item_heights = {
        "item_red": properties["item_red_height"],
        "item_blue": properties["item_blue_length"],
        "item_green": properties["item_green_radius"] * 2.0,
    }

    bins = {
        "bin_a": "table_to_bin_a",
        "bin_b": "table_to_bin_b",
        "reject_bin": "table_to_reject_bin",
    }

    report_lines = [f"scene: {xacro_path}", f"table_top_z: {table_top_z:.3f}"]

    for item, joint in item_joints.items():
        center_z_local = _find_joint_origin(root, joint, properties)[2]
        bottom_z_local = center_z_local - item_heights[item] / 2.0
        bottom_z = table_top_z + bottom_z_local
        ok = math.isclose(bottom_z_local, 0.0, abs_tol=EPS)
        if not ok:
            raise AssertionError(f"{item} bottom local z {bottom_z_local} does not equal tabletop local z 0.0")
        report_lines.append(f"{item}: bottom_z={bottom_z:.3f} local_bottom_z={bottom_z_local:.3f} OK")

    tray_height = properties["tray_height"]
    tray_length = properties["tray_length"]
    tray_width = properties["tray_width"]
    tray_wall_thickness = properties["tray_wall_thickness"]
    for bin_name, joint in bins.items():
        frame_z_local = _find_joint_origin(root, joint, properties)[2]
        tray_bottom_z_local = frame_z_local - tray_height
        tray_bottom_z = table_top_z + tray_bottom_z_local
        if not math.isclose(frame_z_local, tray_height, abs_tol=EPS):
            raise AssertionError(f"{bin_name} frame local z {frame_z_local} does not equal tray_height {tray_height}")
        if not math.isclose(tray_bottom_z_local, 0.0, abs_tol=EPS):
            raise AssertionError(
                f"{bin_name} tray bottom local z {tray_bottom_z_local} does not equal tabletop local z 0.0"
            )
        report_lines.append(f"{bin_name}: tray_bottom_z={tray_bottom_z:.3f} local_bottom_z={tray_bottom_z_local:.3f} OK")
        report_lines.append(f"{bin_name}: frame_at_tray_opening=YES local_frame_z={frame_z_local:.3f}")



    def _radius(name: str) -> float:
        if name == "item_red":
            return math.sqrt((0.04 / 2.0) ** 2 + (0.04 / 2.0) ** 2)
        if name == "item_blue":
            return 0.02
        return properties["item_green_radius"]

    item_xy: dict[str, tuple[float, float]] = {}
    for item, joint in item_joints.items():
        x, y, _ = _find_joint_origin(root, joint, properties)
        item_xy[item] = (x, y)
    item_names = list(item_xy.keys())
    for i, a in enumerate(item_names):
        for b in item_names[i + 1:]:
            ax, ay = item_xy[a]
            bx, by = item_xy[b]
            distance = math.hypot(ax - bx, ay - by)
            min_distance = _radius(a) + _radius(b)
            if distance + EPS < min_distance:
                raise AssertionError(f"pickup items overlap: {a} and {b}")
    report_lines.append("pickup_items_non_overlapping: YES")

    bin_xy: dict[str, tuple[float, float]] = {}
    for bin_name, joint in bins.items():
        x, y, _ = _find_joint_origin(root, joint, properties)
        bin_xy[bin_name] = (x, y)
    bin_names = list(bin_xy.keys())
    for i, a in enumerate(bin_names):
        for b in bin_names[i + 1:]:
            ax, ay = bin_xy[a]
            bx, by = bin_xy[b]
            if abs(ax - bx) + EPS < tray_length and abs(ay - by) + EPS < tray_width:
                raise AssertionError(f"destination trays overlap: {a} and {b}")
    report_lines.append("destination_trays_non_overlapping: YES")

    manifest_path = xacro_path.parents[1] / "sorting_manifest.yaml"
    if manifest_path.exists():
        text = manifest_path.read_text(encoding="utf-8")
        for m in re.finditer(r"release_offset_xyz_m:\s*\[([^\]]+)\]", text):
            coords = [float(x.strip()) for x in m.group(1).split(",")]
            if len(coords) != 3 or coords[2] <= 0.0:
                raise AssertionError(f"invalid non-positive release_offset_xyz_m z: {m.group(0)}")

    if root.find("link[@name='robot_mount_plate']") is None:
        raise AssertionError("robot_mount_plate missing for table-mounted robot")
    report_lines.append("robot_mount_plate_link: OK")
    if root.find("joint[@name='table_to_robot_mount_plate']") is None:
        raise AssertionError("table_to_robot_mount_plate joint missing")

    robot_origin_z = None
    for ur_robot in root.findall(f"{XACRO_NS}ur_robot"):
        origin = ur_robot.find("origin")
        if origin is not None and "xyz" in origin.attrib:
            robot_origin_z = _parse_value(_split_xyz(origin.attrib["xyz"])[2], properties)
            break
    if robot_origin_z is None:
        raise AssertionError("unable to determine robot base origin z")

    if robot_origin_z + EPS < table_top_z:
        raise AssertionError("robot base origin is below table top/floor unexpectedly")
    report_lines.append("robot_mount: OK")

    if root.find("link[@name='camera_link']") is None and root.find("link[@name='camera_frame']") is None:
        raise AssertionError("camera link/frame missing")
    report_lines.append("camera_mount: OK")

    print("\n".join(report_lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
