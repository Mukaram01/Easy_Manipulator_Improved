#!/usr/bin/env python3
"""Regenerate one Workcell Studio Scene3D visual mesh index.

This is the narrow CLI path for refreshing
``scenes/<scene>/generated/scene_visual_mesh_index.json`` without launching the
Workcell Builder GUI.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--workspace-root", default="")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--use-fake-hardware", default="true")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    extractor = repo_root / "scripts" / "extract_scene_urdf_visual_mesh_index.py"
    if not extractor.exists():
        print(f"visual mesh index extractor missing: {extractor}", file=sys.stderr)
        return 2

    cmd = [
        sys.executable,
        str(extractor),
        "--scene",
        args.scene,
        "--workspace-root",
        args.workspace_root,
        "--use-fake-hardware",
        args.use_fake_hardware,
    ]
    print("[generate_scene_visual_mesh_index] running: " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=repo_root).returncode


if __name__ == "__main__":
    raise SystemExit(main())
