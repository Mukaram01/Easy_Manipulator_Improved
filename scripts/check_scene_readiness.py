#!/usr/bin/env python3
"""Offline scene creation readiness checker for existing workcell_builder scenes."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path
from typing import Any

MESH_EXTS = {".stl", ".dae", ".obj"}
SCENE_TEXT_EXTS = {".urdf", ".xacro", ".yaml", ".yml", ".json", ".xml", ".txt", ".srdf"}
FRAME_HINTS = ("world", "base_link", "tool0", "camera_link")


def _discover_scene_packages(scenes_dir: Path) -> list[Path]:
    return sorted([p for p in scenes_dir.iterdir() if p.is_dir()]) if scenes_dir.is_dir() else []


def _extract_mesh_refs(text: str) -> list[str]:
    refs: list[str] = []
    refs.extend(re.findall(r"filename\s*=\s*[\"']([^\"']+)[\"']", text, flags=re.IGNORECASE))
    refs.extend(re.findall(r"(?:package://[^\s\"']+\.(?:stl|dae|obj)|[^\s\"']+\.(?:stl|dae|obj))", text, flags=re.IGNORECASE))
    return refs


def _looks_absolute(ref: str) -> bool:
    lowered = ref.lower()
    return lowered.startswith("/") or re.match(r"^[a-zA-Z]:\\", ref) is not None or "/home/" in lowered


def _asset_name_from_mesh(path: Path) -> str:
    base = re.sub(r"[^a-zA-Z0-9_]+", "_", path.stem.lower()).strip("_")
    return base or "imported_asset"


def _analyze_scene_package(scene_pkg: Path, repo_root: Path) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []

    files = [p for p in scene_pkg.rglob("*") if p.is_file()]
    mesh_files = [p for p in files if p.suffix.lower() in MESH_EXTS]
    urdf_xacro_files = [p for p in files if ".urdf" in p.name.lower() or p.suffix.lower() == ".xacro"]
    text_files = [p for p in files if p.suffix.lower() in SCENE_TEXT_EXTS or any(ext in p.name.lower() for ext in (".urdf", ".xacro", ".srdf"))]

    if (scene_pkg / "package.xml").exists():
        notes.append("ROS scene package detected (package.xml present).")
    else:
        warnings.append("Scene package has no package.xml (may be non-ROS folder).")

    if not urdf_xacro_files:
        errors.append("No URDF/Xacro files found in scene package.")

    frame_hits: dict[str, int] = {frame: 0 for frame in FRAME_HINTS}
    candidate_clues = {"robot": set(), "gripper": set(), "sensor": set()}

    for path in text_files:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue

        for frame in FRAME_HINTS:
            frame_hits[frame] += len(re.findall(rf"\b{re.escape(frame)}\b", text))

        lower = (path.name + "\n" + text[:2000]).lower()
        if any(tok in lower for tok in ("ur5", "ur10", "ur3", "fanuc", "panda", "robot")):
            candidate_clues["robot"].add(path.name)
        if any(tok in lower for tok in ("robotiq", "gripper", "suction", "airpick", "end_effector")):
            candidate_clues["gripper"].add(path.name)
        if any(tok in lower for tok in ("camera", "realsense", "sensor", "rgbd")):
            candidate_clues["sensor"].add(path.name)

        mesh_refs = _extract_mesh_refs(text)
        has_visual = bool(re.search(r"<\s*visual\b|\bvisual\s*:", text, flags=re.IGNORECASE))
        has_collision = bool(re.search(r"<\s*collision\b|\bcollision\s*:", text, flags=re.IGNORECASE))
        if has_visual and not has_collision:
            warnings.append(f"Potential missing collision geometry in {path.relative_to(scene_pkg)}.")

        for ref in mesh_refs:
            if _looks_absolute(ref):
                warnings.append(f"Absolute mesh path found in {path.relative_to(scene_pkg)}: {ref}")
                continue
            if ref.startswith("package://"):
                continue
            candidate = (scene_pkg / ref).resolve()
            if not candidate.exists() and not (repo_root / ref).resolve().exists():
                warnings.append(f"Referenced mesh path not found: {ref} (from {path.relative_to(scene_pkg)}).")

    counts = Counter(p.name.lower() for p in mesh_files)
    duplicate_basenames = sorted([name for name, count in counts.items() if count > 1])
    if duplicate_basenames:
        warnings.append(f"Duplicate mesh basenames in scene package: {', '.join(duplicate_basenames)}")

    return {
        "scene_package": str(scene_pkg),
        "errors": errors,
        "warnings": warnings,
        "notes": notes,
        "counts": {
            "files": len(files),
            "meshes": len(mesh_files),
            "urdf_xacro": len(urdf_xacro_files),
        },
        "frames": {k: v for k, v in frame_hits.items() if v > 0},
        "candidates": {k: sorted(v) for k, v in candidate_clues.items() if v},
        "mesh_extensions_detected": sorted({p.suffix.lower() for p in mesh_files}),
    }


def check_readiness(workcell_root: Path | None, scene_package: Path | None, strict: bool = False) -> dict[str, Any]:
    repo_root = Path(__file__).resolve().parents[1]
    errors: list[str] = []
    warnings: list[str] = []
    notes: list[str] = []
    scene_reports: list[dict[str, Any]] = []

    if workcell_root is not None:
        if not workcell_root.exists() or not workcell_root.is_dir():
            errors.append(f"Workcell root does not exist or is not a directory: {workcell_root}")
        scenes_dir = workcell_root / "scenes"
        assets_dir = workcell_root / "assets"
        if not scenes_dir.is_dir():
            errors.append(f"Missing scenes directory: {scenes_dir}")
        else:
            for pkg in _discover_scene_packages(scenes_dir):
                scene_reports.append(_analyze_scene_package(pkg, repo_root))

        if not assets_dir.is_dir():
            warnings.append(
                f"Assets directory missing at {assets_dir}; existing workcell_builder default assets fallback may be used."
            )

    if scene_package is not None:
        if not scene_package.exists() or not scene_package.is_dir():
            errors.append(f"Scene package path does not exist or is not a directory: {scene_package}")
        else:
            scene_reports.append(_analyze_scene_package(scene_package, repo_root))

    if not scene_reports and not errors:
        warnings.append("No scene packages were inspected.")

    for report in scene_reports:
        errors.extend(report["errors"])
        warnings.extend(report["warnings"])
        notes.extend(report["notes"])

    result = "PASS"
    if errors:
        result = "FAIL"
    elif warnings:
        result = "WARN"

    if strict and warnings and not errors:
        result = "FAIL"

    return {
        "result": result,
        "strict": strict,
        "inputs": {
            "workcell_root": str(workcell_root) if workcell_root else None,
            "scene_package": str(scene_package) if scene_package else None,
        },
        "summary": {
            "scene_packages_checked": len(scene_reports),
            "errors": len(errors),
            "warnings": len(warnings),
        },
        "errors": errors,
        "warnings": warnings,
        "notes": notes,
        "scene_reports": scene_reports,
    }


def _print_human(payload: dict[str, Any]) -> None:
    print(f"RESULT: {payload['result']}")
    print(f"Scene packages checked: {payload['summary']['scene_packages_checked']}")
    for note in payload.get("notes", []):
        print(f"NOTE: {note}")
    for warning in payload.get("warnings", []):
        print(f"WARN: {warning}")
    for error in payload.get("errors", []):
        print(f"FAIL: {error}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell-root", type=Path)
    parser.add_argument("--scene-package", type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.workcell_root is None and args.scene_package is None:
        parser.error("At least one of --workcell-root or --scene-package is required.")

    payload = check_readiness(args.workcell_root, args.scene_package, strict=args.strict)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        _print_human(payload)

    return 0 if payload["result"] == "PASS" or payload["result"] == "WARN" else 1


if __name__ == "__main__":
    raise SystemExit(main())
