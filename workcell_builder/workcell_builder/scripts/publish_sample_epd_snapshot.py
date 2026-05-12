#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CLASSES = ["box", "bottle", "unknown"]

def build_snapshot(camera: str, zone: str, class_label: str) -> dict:
    return {"schema_version": 1, "source": "epd_sample_fake", "runtime_mode": "sample_preview", "camera": camera, "camera_frame": "camera_color_optical_frame", "timestamp_sec": time.time(), "detections": [{"id": f"det_{int(time.time()*1000)}", "class_label": class_label, "confidence": 0.92, "center_px": [320,180], "bbox_px": [280,140,80,70], "estimated_xyz_camera": [0.05,0.02,0.85], "estimated_xyz_world": [0.60,0.00,0.80], "zone_hint": zone, "tracking_id": "track_001"}]}

class SamplePublisher(Node):
    def __init__(self, args):
        super().__init__('publish_sample_epd_snapshot')
        self.args=args; self.i=0
        self.pub=self.create_publisher(String,args.topic,10)
        self.timer=self.create_timer(args.period_s,self.tick)
    def tick(self):
        label = self.args.class_label if self.args.class_label != 'sequence' else CLASSES[self.i % len(CLASSES)]
        self.i += 1
        self.pub.publish(String(data=json.dumps(build_snapshot(self.args.camera, self.args.zone, label))))

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--topic', default='/workcell_studio/epd_detection_snapshot_json')
    ap.add_argument('--camera', default='realsense_d435i_1')
    ap.add_argument('--zone', default='detection_zone_1')
    ap.add_argument('--period-s', type=float, default=2.0)
    ap.add_argument('--class-label', default='sequence', choices=['sequence','box','bottle','unknown'])
    ap.add_argument('--once', action='store_true')
    args=ap.parse_args()
    if args.once:
        print(json.dumps(build_snapshot(args.camera,args.zone,'box')))
        return 0
    rclpy.init(); node=SamplePublisher(args); rclpy.spin(node); node.destroy_node(); rclpy.shutdown(); return 0

if __name__=='__main__':
    raise SystemExit(main())
