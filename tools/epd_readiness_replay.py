"""Publish recorded EPD detection_readiness reports (exact content, original stamps) for non-motion GUI checks.

Usage: python3 tools/epd_readiness_replay.py tests/fixtures/<fixture>.jsonl <interval_s> A_00,B_00,...
"""
import json
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, ReliabilityPolicy

fixture, interval, frames = sys.argv[1], float(sys.argv[2]), sys.argv[3].split(',')
reports = {r['frame']: r for r in map(json.loads, open(fixture).read().splitlines())}
rclpy.init()
node = rclpy.create_node('recorded_epd_readiness_replay')
pub = node.create_publisher(DiagnosticArray, '/easy_perception_deployment/detection_readiness',
                            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
deadline = time.monotonic() + 20
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
for frame in frames:
    r = reports[frame]
    msg = DiagnosticArray()
    msg.header.stamp.sec, msg.header.stamp.nanosec = r['stamp']
    msg.header.frame_id = r['frame_id']
    for s in r['statuses']:
        st = DiagnosticStatus(name=s['name'], message=s['message'], hardware_id=r['hardware_id'],
                              level=bytes([s['level']]))
        st.values = [KeyValue(key=k, value=v) for k, v in s['values'].items()]
        msg.status.append(st)
    pub.publish(msg)
    print(f'{time.time():.2f} published {frame} ({len(msg.status)} detections)', flush=True)
    end = time.monotonic() + interval
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
