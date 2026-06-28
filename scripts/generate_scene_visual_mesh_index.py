#!/usr/bin/env python3
"""Regenerate or repair one Workcell Studio Scene3D visual mesh index."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


if str(Path(__file__).resolve().parents[1]) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from scripts.repair_ur5_scene3d_visual_index import repair_index
except Exception:  # pragma: no cover - optional post-processing safety net
    repair_index = None


def _list_value(payload: dict, key: str) -> list:
    value = payload.get(key)
    return value if isinstance(value, list) else []


def _repair_detail(index_path: Path, links: list[str]) -> str:
    try:
        payload = json.loads(index_path.read_text(encoding="utf-8"))
    except Exception:
        return ", ".join(links) if links else "visual rows normalized"
    added_arm = links or _list_value(payload, "ur5_runtime_repair_added_links")
    added_eef = _list_value(payload, "ur5_runtime_repair_added_end_effector_links")
    reasons = _list_value(payload, "ur5_runtime_repair_reasons")
    if added_arm:
        return "ur5_links=" + ",".join(str(v) for v in added_arm)
    if added_eef:
        return "end_effector_links=" + ",".join(str(v) for v in added_eef)
    if reasons:
        return "reasons=" + ",".join(str(v) for v in reasons)
    return "visual rows normalized"


def _repair_existing_index(index_path: Path) -> int:
    if repair_index is None:
        print("visual mesh index repair helper unavailable", file=sys.stderr)
        return 2
    if not index_path.exists():
        print(f"visual mesh index missing: {index_path}", file=sys.stderr)
        return 2
    changed, links = repair_index(index_path)
    if changed:
        print("[generate_scene_visual_mesh_index] UR5 repair applied: " + _repair_detail(index_path, links), flush=True)
    else:
        print("[generate_scene_visual_mesh_index] existing index already safe for preview", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--workspace-root", default="")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--use-fake-hardware", default="true")
    parser.add_argument(
        "--repair-existing-only",
        action="store_true",
        help="Repair the existing generated/scene_visual_mesh_index.json without rerunning xacro extraction.",
    )
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    index_path = repo_root / "scenes" / args.scene / "generated" / "scene_visual_mesh_index.json"

    if args.repair_existing_only:
        return _repair_existing_index(index_path)

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
    result = subprocess.run(cmd, cwd=repo_root).returncode
    if result != 0:
        return result

    return _repair_existing_index(index_path)


if __name__ == "__main__":
    raise SystemExit(main())
