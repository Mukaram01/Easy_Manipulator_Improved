#!/usr/bin/env python3
"""Inspect existing repository assets and emit inventory reports."""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = REPO_ROOT / "reports"
MESH_EXTS = {".stl", ".dae", ".obj"}
ROBOT_EXTS = {".urdf", ".xacro"}
SCENE_EXTS = {".yaml", ".yml"}


def _looks_like_asset_dir(path: Path) -> bool:
    text = str(path).lower()
    hints = ("asset", "assets", "mesh", "meshes", "model", "models", "description", "urdf", "xacro", "scene", "world")
    return any(hint in text for hint in hints)


def _guess_type(path: Path) -> str:
    text = str(path).lower()
    rules = [
        ("robot", ("robot", "ur", "fanuc", "panda")),
        ("end_effector", ("end_effector", "gripper", "suction", "airpick", "robotiq")),
        ("sensor", ("sensor", "camera", "realsense")),
        ("table", ("table",)),
        ("workbench", ("workbench", "bench")),
        ("bin", ("bin", "tray")),
        ("conveyor", ("conveyor",)),
        ("fixture", ("fixture",)),
    ]
    for label, keys in rules:
        if any(key in text for key in keys):
            return label
    return "unknown"


def _pkg_for(path: Path, package_dirs: dict[Path, str]) -> str | None:
    matches = [(pkg_path, name) for pkg_path, name in package_dirs.items() if pkg_path in path.parents or pkg_path == path]
    if not matches:
        return None
    return max(matches, key=lambda item: len(item[0].parts))[1]


def collect_inventory(repo_root: Path) -> dict[str, Any]:
    package_dirs: dict[Path, str] = {}
    for pkg_xml in repo_root.rglob("package.xml"):
        package_dirs[pkg_xml.parent] = pkg_xml.parent.name

    entries: list[dict[str, Any]] = []
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)

    for path in repo_root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(repo_root)
        if rel.parts and rel.parts[0] == ".git":
            continue
        if "__pycache__" in rel.parts:
            continue

        suffix = path.suffix.lower()
        category = None
        if suffix in MESH_EXTS:
            category = "mesh"
        elif suffix in ROBOT_EXTS:
            category = "urdf_xacro"
        elif suffix in SCENE_EXTS and ("scene" in str(rel).lower() or "config" in str(rel).lower()):
            category = "scene_yaml"
        elif path.name == "package.xml":
            category = "package_xml"

        if category is None and not _looks_like_asset_dir(rel.parent):
            continue
        if category is None:
            continue

        pkg = _pkg_for(path, package_dirs)
        source_group = str(rel.parent)
        item = {
            "relative_path": str(rel),
            "file_name": path.name,
            "file_type": suffix.lstrip(".") or path.name,
            "category": category,
            "asset_type_guess": _guess_type(rel),
            "package": pkg,
            "package_uri": f"package://{pkg}/{rel}" if pkg else None,
            "group": source_group,
        }
        entries.append(item)
        grouped[source_group].append(item)

    counts = Counter(item["category"] for item in entries)
    return {
        "schema_version": "asset_inventory/v1",
        "repo_root": str(repo_root),
        "totals": {
            "files": len(entries),
            "meshes": counts.get("mesh", 0),
            "urdf_xacro": counts.get("urdf_xacro", 0),
            "scene_yaml": counts.get("scene_yaml", 0),
            "package_xml": counts.get("package_xml", 0),
            "packages": len(package_dirs),
        },
        "groups": [{"group": group, "items": items} for group, items in sorted(grouped.items())],
        "entries": sorted(entries, key=lambda item: item["relative_path"]),
    }


def _render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# Asset Inventory Report",
        "",
        f"- Schema: `{payload['schema_version']}`",
        f"- Total tracked files: `{payload['totals']['files']}`",
        f"- Meshes: `{payload['totals']['meshes']}`",
        f"- URDF/Xacro: `{payload['totals']['urdf_xacro']}`",
        f"- Scene YAML: `{payload['totals']['scene_yaml']}`",
        f"- package.xml files: `{payload['totals']['package_xml']}`",
        "",
        "## Grouped inventory",
        "",
    ]
    for block in payload["groups"]:
        lines.append(f"### {block['group']}")
        lines.append("")
        lines.append("| Asset Type Guess | Category | File | Package | Package URI |")
        lines.append("|---|---|---|---|---|")
        for item in block["items"]:
            lines.append(
                f"| {item['asset_type_guess']} | {item['category']} | `{item['relative_path']}` | "
                f"`{item['package'] or '-'}` | `{item['package_uri'] or '-'}` |"
            )
        lines.append("")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--markdown", type=Path, default=REPORT_DIR / "asset_inventory.md")
    parser.add_argument("--json", dest="json_path", type=Path, default=REPORT_DIR / "asset_inventory.json")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    payload = collect_inventory(args.repo_root.resolve())
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(_render_markdown(payload), encoding="utf-8")
    args.json_path.parent.mkdir(parents=True, exist_ok=True)
    args.json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if not args.quiet:
        print(f"PASS: wrote {args.markdown}")
        print(f"PASS: wrote {args.json_path}")
        print(f"SUMMARY: files={payload['totals']['files']} meshes={payload['totals']['meshes']} urdf_xacro={payload['totals']['urdf_xacro']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
