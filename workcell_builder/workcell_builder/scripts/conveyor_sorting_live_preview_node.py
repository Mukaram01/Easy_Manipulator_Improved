#!/usr/bin/env python3
from __future__ import annotations
import json, math, time
from pathlib import Path
from dataclasses import dataclass
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker

@dataclass
class PreviewState:
    class_label: str = "unknown"
    detection_zone: str = "detection_zone_1"
    pick_zone: str = "pick_zone_1"
    selected_place_zone: str = "reject_zone"
    time_to_pick_s: float = 0.0
    pick_ready: bool = False


def safe_parse_snapshot(raw: str):
    try:
        return json.loads(raw), None
    except Exception as exc:
        return None, str(exc)


class ConveyorSortingLivePreviewNode(Node):
    def __init__(self):
        super().__init__('conveyor_sorting_live_preview_node')
        self.declare_parameter('scenario_dir', '')
        self.declare_parameter('epd_snapshot_topic', '/workcell_studio/epd_detection_snapshot_json')
        self.declare_parameter('marker_topic', '/workcell_studio/conveyor_sorting_preview_markers')
        self.state = PreviewState()
        self.last_detection_ts = time.time()
        self.progress = 0.0
        self.class_routes = {'box': 'place_zone_box', 'bottle': 'place_zone_bottle', 'unknown': 'reject_zone'}
        self.status_pub = self.create_publisher(String, '/workcell_studio/conveyor_sorting_preview_status', 10)
        self.marker_pub = self.create_publisher(Marker, self.get_parameter('marker_topic').value, 20)
        self.sub = self.create_subscription(String, self.get_parameter('epd_snapshot_topic').value, self.on_snapshot, 20)
        self.timer = self.create_timer(0.2, self.on_timer)

    def on_snapshot(self, msg: String):
        payload, err = safe_parse_snapshot(msg.data)
        if err:
            self.get_logger().warning(f"Malformed EPD snapshot JSON: {err}")
            return
        detections = payload.get('detections', [])
        if not detections:
            return
        d = detections[0]
        self.state.class_label = d.get('class_label', 'unknown')
        self.state.detection_zone = d.get('zone_hint', 'detection_zone_1')
        self.state.selected_place_zone = self.class_routes.get(self.state.class_label, self.class_routes['unknown'])
        self.progress = 0.0
        self.last_detection_ts = time.time()

    def compute(self):
        elapsed = max(0.0, time.time() - self.last_detection_ts)
        speed = 0.125
        self.progress = min(1.0, elapsed * speed)
        remain = max(0.0, (1.0 - self.progress) / speed)
        self.state.time_to_pick_s = round(remain, 2)
        self.state.pick_ready = self.progress >= 0.95

    def on_timer(self):
        self.compute()
        status = {
            'scenario': 'conveyor_sorting_live_epd_preview', 'detections': 1,
            'class_label': self.state.class_label,
            'detection_zone': self.state.detection_zone,
            'pick_zone': self.state.pick_zone,
            'selected_place_zone': self.state.selected_place_zone,
            'time_to_pick_s': self.state.time_to_pick_s,
            'pick_ready': self.state.pick_ready,
            'robot_motion_commanded': False,
            'gripper_command_sent': False,
            'conveyor_command_sent': False,
            'sample_detection': True,
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        self.publish_markers()
        self.write_artifacts(status)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        m = Marker(); m.header.frame_id='world'; m.header.stamp=now; m.ns='conveyor_sorting_preview'; m.id=1; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.x = 0.1 + 0.8 * self.progress; m.pose.position.y = 0.0; m.pose.position.z = 0.9
        m.scale.x = m.scale.y = m.scale.z = 0.05; m.color.a = 1.0; m.color.g = 1.0
        self.marker_pub.publish(m)

    def write_artifacts(self, status: dict):
        scenario_dir = Path(self.get_parameter('scenario_dir').value or '.')
        pdir = scenario_dir / 'preview'
        pdir.mkdir(parents=True, exist_ok=True)
        (pdir / 'live_conveyor_sorting_status.json').write_text(json.dumps(status, indent=2))
        (pdir / 'live_conveyor_sorting_status.yaml').write_text(yaml.safe_dump(status, sort_keys=False))
        (pdir / 'live_task_intent_preview.yaml').write_text(yaml.safe_dump({'task': 'pick_place_preview', 'selected_place_zone': self.state.selected_place_zone, 'pick_ready': self.state.pick_ready}))
        (pdir / 'live_emd_grasp_planner_request.yaml').write_text(yaml.safe_dump({'request': 'preview_only', 'robot_motion_commanded': False, 'gripper_command_sent': False, 'conveyor_command_sent': False}))


def main():
    rclpy.init()
    node = ConveyorSortingLivePreviewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
