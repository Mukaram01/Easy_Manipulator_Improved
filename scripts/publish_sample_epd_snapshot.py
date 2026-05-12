#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, time

def build_snapshot(camera: str, zone: str) -> dict:
    return {
      "schema_version": 1,
      "source": "epd_live",
      "runtime_mode": "live_adapter_metadata_only",
      "camera": camera,
      "camera_frame": "camera_color_optical_frame",
      "timestamp_sec": time.time(),
      "detections": [{"id":"det_001","class_label":"box","confidence":0.92,"center_px":[320,180],"bbox_px":[280,140,80,70],"estimated_xyz_camera":[0.05,0.02,0.85],"estimated_xyz_world":[0.60,0.00,0.80],"zone_hint":zone,"tracking_id":"track_001"}]
    }

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--topic', default='/workcell_studio/epd_detection_snapshot_json')
    ap.add_argument('--camera', default='realsense_d435i_1')
    ap.add_argument('--zone', default='detection_zone_1')
    ap.add_argument('--once', action='store_true')
    args=ap.parse_args()
    payload=json.dumps(build_snapshot(args.camera,args.zone))
    print(f"Sample publisher stub topic={args.topic} payload={payload}")
    return 0

if __name__=='__main__':
    raise SystemExit(main())
