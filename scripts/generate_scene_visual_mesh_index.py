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

from scripts.scene3d_visual_index_report import run_visual_index_postprocess  # noqa: E402


def _list_value(data: dict, key: str) -> list:
    value = data.get(key)
    return value if isinstance(value, list) else []


def _format_repair_detail(index_path: Path, changed: bool, added_links: list[str] | tuple[str, ...] | None) -> str:
    data = json.loads(index_path.read_text(encoding="utf-8")) if index_path.exists() else {}
    added_arm = list(added_links) if added_links else _list_value(data, "ur5_runtime_repair_added_links")
    added_eef = _list_value(data, "ur5_runtime_repair_added_end_effector_links")
    if changed and added_arm:
        return "ur5_links=" + ",".join(str(v) for v in added_arm)
    if changed and added_eef:
        return "end_effector_links=" + ",".join(str(v) for v in added_eef)
    if changed:
        return "visual_rows_normalized"
    return "already_safe"


def _print_repair_result(result: dict) -> int:
    if result.get("postprocess_error"):
        print("visual mesh index repair failed: " + str(result["postprocess_error"]), file=sys.stderr)
        return 2
    if result.get("postprocess_changed"):
        print(
            "[generate_scene_visual_mesh_index] UR5 repair applied: "
            + str(result.get("postprocess_detail") or "visual rows normalized"),
            flush=True,
        )
    else:
        print("[generate_scene_visual_mesh_index] existing index already safe for preview", flush=True)
    return 0


def _repair_existing_index(index_path: Path) -> int:
    """Repair one existing visual index file.

    This compatibility entry point is intentionally kept for older tests and
    callers that repair a concrete ``scene_visual_mesh_index.json`` path rather
    than a full scene directory.
    """
    try:
        from scripts.repair_ur5_scene3d_visual_index import repair_index  # noqa: PLC0415
    except Exception as exc:  # pragma: no cover
        return _print_repair_result(
            {
                "postprocess_changed": False,
                "postprocess_error": f"repair module unavailable: {exc}",
                "postprocess_detail": "",
            }
        )

    try:
        changed, added_links = repair_index(index_path)
        detail = _format_repair_detail(index_path, bool(changed), added_links)
        return _print_repair_result(
            {
                "postprocess_changed": bool(changed),
                "postprocess_error": "",
                "postprocess_detail": detail,
            }
        )
    except Exception as exc:  # pragma: no cover
        return _print_repair_result(
            {
                "postprocess_changed": False,
                "postprocess_error": str(exc),
                "postprocess_detail": "",
            }
        )


def _repair_scene_index(scene_dir: Path) -> int:
    result = run_visual_index_postprocess(scene_dir)
    return _print_repair_result(result)


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
    scene_dir = repo_root / "scenes" / args.scene

    if args.repair_existing_only:
        return _repair_scene_index(scene_dir)

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

    return _repair_scene_index(scene_dir)


if __name__ == "__main__":
    raise SystemExit(main())
