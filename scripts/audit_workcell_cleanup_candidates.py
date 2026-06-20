#!/usr/bin/env python3
"""Audit obvious cleanup candidates without deleting anything.

This utility classifies local workspace artifacts so maintainers can review
candidate cleanup items while protecting scene/runtime files by default.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, TypedDict


class CleanupClassification(TypedDict):
    path: str
    category: str
    confidence: str
    safe_to_delete_now: bool
    reason: str


PROTECTED_EXTENSIONS = {".launch.py", ".xacro", ".urdf", ".stl", ".dae", ".obj", ".ply", ".mesh"}
PROTECTED_NAMES = {
    "scene_manifest.yaml",
    "scene_manifest.yml",
    "environment.yaml",
    "environment.yml",
    "cell_definition.yaml",
    "cell_definition.yml",
}
SAFETY_PATH_PARTS = {"launch", "urdf", "meshes", "mesh", "config", "controllers", "runtime", "safety"}


def _rel_parts(path: Path, repo_root: Path | None = None) -> tuple[str, ...]:
    candidate = path
    if repo_root is not None:
        try:
            candidate = path.resolve().relative_to(repo_root.resolve())
        except ValueError:
            candidate = path
    return candidate.as_posix().split("/")


def _has_protected_suffix(path: Path) -> bool:
    text = path.name
    return any(text.endswith(suffix) for suffix in PROTECTED_EXTENSIONS)


def classify_cleanup_candidate(path: Path | str, repo_root: Path | str | None = None) -> CleanupClassification:
    """Classify a path as a reviewable cleanup candidate or protected file."""

    path = Path(path)
    root = Path(repo_root) if repo_root is not None else None
    parts = _rel_parts(path, root)
    rel = "/".join(parts)
    name = path.name
    lowered_name = name.lower()
    lowered_parts = tuple(part.lower() for part in parts)

    if "__pycache__" in lowered_parts or lowered_name.endswith((".pyc", ".pyo")):
        return {
            "path": rel,
            "category": "python_cache",
            "confidence": "high",
            "safe_to_delete_now": True,
            "reason": "Python bytecode/cache artifact.",
        }

    if ".pytest_cache" in lowered_parts:
        return {
            "path": rel,
            "category": "python_cache",
            "confidence": "high",
            "safe_to_delete_now": True,
            "reason": "Pytest cache artifact.",
        }

    if lowered_name.endswith((".stdout.log", ".stderr.log", ".log")):
        return {
            "path": rel,
            "category": "generated_logs",
            "confidence": "medium",
            "safe_to_delete_now": True,
            "reason": "Generated log output.",
        }

    if lowered_name.startswith("scene3d_gui_smoke_") and lowered_name.endswith(".json"):
        return {
            "path": rel,
            "category": "generated_smoke_outputs",
            "confidence": "medium",
            "safe_to_delete_now": True,
            "reason": "Scene3D GUI smoke-test output.",
        }

    if lowered_name.endswith((".bak", ".backup", "~", ".orig")):
        return {
            "path": rel,
            "category": "temp_backup_files",
            "confidence": "medium",
            "safe_to_delete_now": True,
            "reason": "Temporary backup file.",
        }

    if name in PROTECTED_NAMES or _has_protected_suffix(path) or any(part in SAFETY_PATH_PARTS for part in lowered_parts):
        return {
            "path": rel,
            "category": "protected_scene_runtime_asset",
            "confidence": "high",
            "safe_to_delete_now": False,
            "reason": "Scene, launch, mesh, URDF/Xacro, config, runtime, or safety-related path.",
        }

    return {
        "path": rel,
        "category": "unknown_review_required",
        "confidence": "low",
        "safe_to_delete_now": False,
        "reason": "No cleanup classification matched; manual review required.",
    }


def audit_paths(paths: Iterable[Path | str], repo_root: Path | str | None = None) -> list[CleanupClassification]:
    """Classify multiple paths without mutating the filesystem."""

    return [classify_cleanup_candidate(path, repo_root) for path in paths]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="Paths to classify. No files are deleted or modified.")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd(), help="Repository root used for relative output paths.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    results = audit_paths(args.paths, args.repo_root)
    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        for result in results:
            print(
                f"{result['path']}: {result['category']} "
                f"confidence={result['confidence']} safe_to_delete_now={result['safe_to_delete_now']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
