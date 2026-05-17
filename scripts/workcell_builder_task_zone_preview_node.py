#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _load_task_zones(preview_dir: Path | None, environment_yaml: Path | None) -> list[dict[str, Any]]:
    if preview_dir:
        preview_yaml = preview_dir / 'task_zone_preview.yaml'
        if preview_yaml.exists():
            data = _load_yaml(preview_yaml)
            zones = data.get('task_zones', [])
            return zones if isinstance(zones, list) else []
    if environment_yaml and environment_yaml.exists():
        data = _load_yaml(environment_yaml)
        zones = data.get('task_zones', [])
        return zones if isinstance(zones, list) else []
    return []


class TaskZonePreviewNode(Node):
    def __init__(self, zones: list[dict[str, Any]], frame_id: str, show_axes: bool) -> None:
        super().__init__('workcell_builder_task_zone_preview')
        self._zones = zones
        self._frame_id = frame_id
        self._show_axes = show_axes
        self._pub = self.create_publisher(MarkerArray, 'task_zone_markers', 10)
        self.create_timer(0.5, self._publish)

    def _publish(self) -> None:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for idx, zone in enumerate(self._zones):
            name = str(zone.get('name', f'zone_{idx+1}'))
            center = zone.get('center', [0.0, 0.0, 0.0])
            size = zone.get('size', [0.2, 0.2, 0.1])
            if not (isinstance(center, list) and len(center) >= 3 and isinstance(size, list) and len(size) >= 3):
                continue
            color = zone.get('color_rgba', [0.1, 0.6, 0.95, 0.25])
            if not (isinstance(color, list) and len(color) >= 4):
                color = [0.1, 0.6, 0.95, 0.25]

            box = Marker()
            box.header.frame_id = self._frame_id
            box.header.stamp = stamp
            box.ns = 'task_zone_box'
            box.id = marker_id
            marker_id += 1
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(center[0])
            box.pose.position.y = float(center[1])
            box.pose.position.z = float(center[2])
            box.pose.orientation.w = 1.0
            box.scale.x = max(float(size[0]), 1e-3)
            box.scale.y = max(float(size[1]), 1e-3)
            box.scale.z = max(float(size[2]), 1e-3)
            box.color.r = float(color[0]); box.color.g = float(color[1]); box.color.b = float(color[2]); box.color.a = float(color[3])
            arr.markers.append(box)

            wire = Marker()
            wire.header.frame_id = self._frame_id
            wire.header.stamp = stamp
            wire.ns = 'task_zone_wire'
            wire.id = marker_id
            marker_id += 1
            wire.type = Marker.LINE_LIST
            wire.action = Marker.ADD
            wire.pose.orientation.w = 1.0
            wire.scale.x = 0.01
            wire.color.r = box.color.r; wire.color.g = box.color.g; wire.color.b = box.color.b; wire.color.a = 0.9
            sx, sy, sz = box.scale.x / 2.0, box.scale.y / 2.0, box.scale.z / 2.0
            cx, cy, cz = box.pose.position.x, box.pose.position.y, box.pose.position.z
            corners = [
                (cx-sx, cy-sy, cz-sz), (cx+sx, cy-sy, cz-sz), (cx+sx, cy+sy, cz-sz), (cx-sx, cy+sy, cz-sz),
                (cx-sx, cy-sy, cz+sz), (cx+sx, cy-sy, cz+sz), (cx+sx, cy+sy, cz+sz), (cx-sx, cy+sy, cz+sz),
            ]
            edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
            for a, b in edges:
                pa = Point(x=corners[a][0], y=corners[a][1], z=corners[a][2])
                pb = Point(x=corners[b][0], y=corners[b][1], z=corners[b][2])
                wire.points.extend([pa, pb])
            arr.markers.append(wire)

            text = Marker()
            text.header.frame_id = self._frame_id
            text.header.stamp = stamp
            text.ns = 'task_zone_label'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = cx; text.pose.position.y = cy; text.pose.position.z = cz + sz + 0.06
            text.pose.orientation.w = 1.0
            text.scale.z = 0.08
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0
            text.text = name
            arr.markers.append(text)

            if self._show_axes:
                for axis, rgb in [('x', (1.0, 0.2, 0.2)), ('y', (0.2, 1.0, 0.2)), ('z', (0.2, 0.4, 1.0))]:
                    ar = Marker()
                    ar.header.frame_id = self._frame_id
                    ar.header.stamp = stamp
                    ar.ns = 'task_zone_axis'
                    ar.id = marker_id
                    marker_id += 1
                    ar.type = Marker.ARROW
                    ar.action = Marker.ADD
                    ar.pose.orientation.w = 1.0
                    ar.scale.x = 0.2; ar.scale.y = 0.02; ar.scale.z = 0.03
                    ar.color.r, ar.color.g, ar.color.b = rgb
                    ar.color.a = 0.95
                    ar.points.append(Point(x=cx, y=cy, z=cz))
                    if axis == 'x':
                        ar.points.append(Point(x=cx + 0.2, y=cy, z=cz))
                    elif axis == 'y':
                        ar.points.append(Point(x=cx, y=cy + 0.2, z=cz))
                    else:
                        ar.points.append(Point(x=cx, y=cy, z=cz + 0.2))
                    arr.markers.append(ar)

        self._pub.publish(arr)


def main() -> int:
    parser = argparse.ArgumentParser(description='Task-zone RViz preview marker publisher (visual-only).')
    parser.add_argument('--preview-dir', default='')
    parser.add_argument('--environment-yaml', default='')
    parser.add_argument('--frame-id', default='world')
    parser.add_argument('--show-axes', action='store_true')
    args = parser.parse_args()

    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    environment_yaml = Path(args.environment_yaml) if args.environment_yaml else None
    zones = _load_task_zones(preview_dir, environment_yaml)

    rclpy.init()
    node = TaskZonePreviewNode(zones=zones, frame_id=args.frame_id, show_axes=args.show_axes)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
