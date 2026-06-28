#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUPPORTED_SCENES = ["ur5_2f_test", "ur5_3f_test", "ur10_2f_test", "ur3_suction_test", "ur5_airpick4_test", "suction_test"]

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.scene3d_visual_index_report import (  # noqa: E402
    format_regeneration_row,
    run_visual_index_postprocess,
    summarize_visual_index,
)


def parse():
    p = argparse.ArgumentParser()
    p.add_argument("--repo-root", type=Path, default=ROOT, help="Repository root that contains scripts/ and scenes/.")
    p.add_argument("--workspace-root", type=Path, help="Optional ROS workspace root forwarded to the extractor.")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--all", action="store_true")
    g.add_argument("--scene")
    p.add_argument("--portable", action="store_true")
    p.add_argument("--prefer-xacro-expanded", action="store_true", default=True)
    p.add_argument("--fallback-best-effort", action="store_true", default=True)
    p.add_argument("--repair-existing-only", action="store_true", help="Repair existing generated indexes without rerunning extraction.")
    p.add_argument("--fail-on-unexpanded", action="store_true")
    p.add_argument("--xacro-arg", action="append", default=[])
    p.add_argument("--fail-on-unsafe", action="store_true")
    return p.parse_args()


def scene_list(args):
    scenes_root = args.repo_root / "scenes"
    if args.scene:
        return [scenes_root / args.scene]
    return [scenes_root / s for s in SUPPORTED_SCENES if (scenes_root / s).exists()]


def extraction_cmd(args, scene, extractor):
    cmd = ["python3", str(extractor), "--scene", scene.name, "--prefer-xacro-expanded"]
    if args.workspace_root is not None:
        cmd += ["--workspace-root", str(args.workspace_root)]
    for xacro_arg in args.xacro_arg:
        cmd += ["--xacro-arg", xacro_arg]
    if args.fail_on_unexpanded:
        cmd.append("--fail-on-unexpanded")
    return cmd


def process_scene(args, scene, extractor):
    if not args.repair_existing_only:
        subprocess.run(extraction_cmd(args, scene, extractor), check=False)
    row = summarize_visual_index(scene, args.repo_root)
    row.update(run_visual_index_postprocess(scene))
    print(format_regeneration_row(row))
    return row


def main():
    args = parse()
    args.repo_root = args.repo_root.resolve()
    extractor = args.repo_root / "scripts/extract_scene_urdf_visual_mesh_index.py"
    rows = [process_scene(args, scene, extractor) for scene in scene_list(args)]
    out = args.repo_root / "build/workcell_studio/visual_mesh_index_regeneration_report.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"schema": "workcell_studio_visual_mesh_index_regeneration/v3", "scenes": rows, "summary": {"scene_count": len(rows)}}, indent=2) + "\n")
    print(out)
    if args.fail_on_unsafe and any(row["status"] != "PASS" for row in rows):
        return 1
    if args.fail_on_unexpanded and any(row["extraction_mode"] != "xacro_expanded" for row in rows):
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
