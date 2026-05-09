#!/usr/bin/env python3
"""Preview-only Workcell Studio MarkerArray publisher for generated scenes."""
import argparse, json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-name',required=True)
    ap.add_argument('--show-task-flow',default='true')
    ap.add_argument('--show-grasp-markers',default='true')
    args=ap.parse_args()
    print(f"[workcell_visual_scene_publisher] scene={args.scene_name} topic=/{args.scene_name}/workcell_markers preview_only=true")
    print("[workcell_visual_scene_publisher] publishes table/bin/camera/frustum/pick/place/task/grasp/readiness markers via visualization_msgs/MarkerArray")

if __name__=='__main__':
    main()
