#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from scripts.repair_scene3d_ur5_spacing import repair_index


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--scene", action="append", default=[])
    parser.add_argument(
        "--write-debug-repair",
        action="store_true",
        help="Rewrite generated Scene3D preview indexes. Default is read-only diagnostics.",
    )
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    scenes_root = repo_root / "scenes"
    scene_names = args.scene or [p.name for p in scenes_root.iterdir() if p.is_dir()]

    repaired = 0
    unchanged = 0
    missing = 0
    for scene_name in sorted(scene_names):
        index_path = scenes_root / scene_name / "generated" / "scene_visual_mesh_index.json"
        if not index_path.exists():
            missing += 1
            print(f"{scene_name}: missing scene_visual_mesh_index.json")
            continue
        changed, reasons = repair_index(index_path, write_debug_repair=args.write_debug_repair)
        if changed:
            repaired += 1
            action = "wrote Debug 3D Preview repair" if args.write_debug_repair else "diagnostic only"
            print(f"{scene_name}: {action} {';'.join(reasons)} (source-of-truth layout/planning artifacts unchanged)")
        else:
            unchanged += 1
            print(f"{scene_name}: already safe")

    label = "repaired" if args.write_debug_repair else "would_repair"
    print(f"summary: {label}={repaired} unchanged={unchanged} missing={missing}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
