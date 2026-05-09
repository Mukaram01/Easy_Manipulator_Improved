#!/usr/bin/env python3
"""Preview-only static planning-scene collision object publisher."""
import argparse

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scene-name',required=True)
    args=ap.parse_args()
    print(f"[workcell_collision_scene_publisher] scene={args.scene_name} static_world_only=true")
    print("[workcell_collision_scene_publisher] adds table/bin/fixture/conveyor primitives when MoveIt planning scene is available")

if __name__=='__main__':
    main()
