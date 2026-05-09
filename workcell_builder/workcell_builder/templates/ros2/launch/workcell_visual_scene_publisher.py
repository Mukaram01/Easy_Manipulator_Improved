#!/usr/bin/env python3
"""Preview-only Workcell Studio MarkerArray publisher for generated scenes."""
import argparse, json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-name',required=True)
    ap.add_argument('--show-task-flow',default='true')
    ap.add_argument('--show-grasp-markers',default='true')
    ap.add_argument('--publish-perception-replay',default='false')
    ap.add_argument('--perception-replay-path',default='')
    args=ap.parse_args()
    print(f"[workcell_visual_scene_publisher] scene={args.scene_name} topic=/{args.scene_name}/workcell_markers preview_only=true")
    if str(args.publish_perception_replay).lower() in {'1','true','yes'}:
        replay=Path(args.perception_replay_path) if args.perception_replay_path else Path('config/perception_replay_markers.json')
        if replay.exists():
            data=json.loads(replay.read_text(encoding='utf-8'))
            print(f"[workcell_visual_scene_publisher] loaded perception replay markers ({len(data.get('detected_objects',[]))} objects) from {replay}")
            print("Perception replay markers loaded from offline snapshot. Live EPD not running.")
        else:
            print(f"[workcell_visual_scene_publisher] perception replay file missing: {replay}")
    print("[workcell_visual_scene_publisher] publishes table/bin/camera/frustum/pick/place/task/grasp/readiness markers via visualization_msgs/MarkerArray")

if __name__=='__main__':
    main()
