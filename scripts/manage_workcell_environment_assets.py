#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import struct
import sys
from pathlib import Path
from typing import Any, Iterable

import yaml

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json"
REGISTRY = ROOT / "config/workcell_studio_asset_replacements.yaml"
SUPPORTED_VISUAL_EXTENSIONS = {".stl", ".dae", ".obj", ".glb", ".gltf"}
TEXT_EXTENSIONS = {
    ".c", ".cc", ".cpp", ".h", ".hpp", ".json", ".md", ".py", ".txt", ".urdf", ".xacro", ".xml", ".yaml", ".yml"
}
SKIP_DIRS = {".git", "build", "install", "log", "node_modules", "__pycache__", ".pytest_cache"}
SAFE_ID = re.compile(r"^[a-z][a-z0-9_]*$")


def _relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return str(path.resolve())


def _load_profile() -> list[dict[str, Any]]:
    data = json.loads(PROFILE.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError(f"{_relative(PROFILE)} must contain a JSON list")
    return [item for item in data if isinstance(item, dict)]


def _load_registry() -> dict[str, Any]:
    data = yaml.safe_load(REGISTRY.read_text(encoding="utf-8")) or {}
    if data.get("schema_version") != "workcell_studio_asset_replacements/v1":
        raise ValueError(f"unsupported replacement registry schema in {_relative(REGISTRY)}")
    assets = data.get("assets")
    if not isinstance(assets, dict):
        raise ValueError("replacement registry must contain an assets mapping")
    return data


def _walk_text_files() -> Iterable[Path]:
    for path in ROOT.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in TEXT_EXTENSIONS:
            continue
        try:
            rel_parts = path.relative_to(ROOT).parts
        except ValueError:
            continue
        if any(part in SKIP_DIRS for part in rel_parts):
            continue
        yield path


def _reference_locations(needle: str, own_package: Path | None = None) -> list[str]:
    refs: list[str] = []
    if not needle:
        return refs
    for path in _walk_text_files():
        if own_package is not None:
            try:
                path.resolve().relative_to(own_package.resolve())
                continue
            except ValueError:
                pass
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if needle in text:
            refs.append(_relative(path))
    return sorted(set(refs))


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _inspect_stl(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "format": path.suffix.lower().lstrip("."),
        "triangle_count": None,
        "ascii": None,
        "zero_normal_count": None,
    }
    if path.suffix.lower() != ".stl" or not path.is_file():
        return result
    data = path.read_bytes()
    stripped = data.lstrip()
    looks_ascii = stripped.startswith(b"solid") and b"facet normal" in data[: min(len(data), 1024 * 1024)]
    result["ascii"] = looks_ascii
    if looks_ascii:
        text = data.decode("utf-8", errors="ignore")
        normals = re.findall(r"facet\s+normal\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", text)
        result["triangle_count"] = len(normals)
        result["zero_normal_count"] = sum(
            1 for normal in normals if all(abs(float(component)) < 1e-12 for component in normal)
        )
    elif len(data) >= 84:
        result["triangle_count"] = struct.unpack("<I", data[80:84])[0]
        result["zero_normal_count"] = None
    return result


def build_audit() -> dict[str, Any]:
    profile = _load_profile()
    registry = _load_registry()
    registered = registry["assets"]
    rows: list[dict[str, Any]] = []

    for item in profile:
        asset_id = str(item.get("asset_id") or "")
        replacement = registered.get(asset_id, {})
        mesh_rel = str(item.get("mesh_path") or "")
        urdf_rel = str(item.get("urdf_path") or "")
        mesh = ROOT / mesh_rel if mesh_rel else Path()
        urdf = ROOT / urdf_rel if urdf_rel else Path()
        own_package = mesh.parent.parent if mesh_rel and len(mesh.parents) >= 2 else None
        refs = _reference_locations(mesh_rel, own_package=own_package)
        filename_refs = _reference_locations(mesh.name, own_package=own_package) if mesh_rel else []
        references = sorted(set(refs + filename_refs))
        source_note = str(item.get("source_note") or "").lower()
        current_quality = str(replacement.get("current_quality") or ("placeholder" if "placeholder" in source_note else "unclassified"))
        action = str(replacement.get("action") or "review")
        status = str(replacement.get("replacement_status") or "unregistered")
        stl = _inspect_stl(mesh)
        safe_to_delete = bool(
            mesh_rel
            and mesh.is_file()
            and current_quality == "placeholder"
            and status in {"replaced", "delete_when_unreferenced"}
            and not references
        )
        rows.append(
            {
                "asset_id": asset_id,
                "label": item.get("label"),
                "category": item.get("category"),
                "mesh_path": mesh_rel,
                "mesh_exists": bool(mesh_rel and mesh.is_file()),
                "mesh_size_bytes": mesh.stat().st_size if mesh_rel and mesh.is_file() else None,
                "mesh_sha256": _sha256(mesh) if mesh_rel else None,
                "urdf_path": urdf_rel,
                "urdf_exists": bool(urdf_rel and urdf.is_file()),
                "current_quality": current_quality,
                "action": action,
                "replacement_status": status,
                "expected_dimensions_m": replacement.get("expected_dimensions_m") or item.get("default_dimensions_m"),
                "canonical_visual_uri": replacement.get("canonical_visual_uri"),
                "references_outside_asset_package": references,
                "safe_to_delete_now": safe_to_delete,
                "mesh_inspection": stl,
            }
        )

    missing_registry = sorted(
        str(item.get("asset_id"))
        for item in profile
        if "placeholder" in str(item.get("source_note") or "").lower()
        and str(item.get("asset_id")) not in registered
    )
    awaiting_models = sorted(row["asset_id"] for row in rows if row["replacement_status"] == "awaiting_model")
    safe_deletes = sorted(row["mesh_path"] for row in rows if row["safe_to_delete_now"])
    return {
        "schema_version": "workcell_studio_environment_asset_audit/v1",
        "profile": _relative(PROFILE),
        "replacement_registry": _relative(REGISTRY),
        "asset_count": len(rows),
        "placeholder_count": sum(row["current_quality"] == "placeholder" for row in rows),
        "awaiting_model_asset_ids": awaiting_models,
        "missing_replacement_registry_entries": missing_registry,
        "safe_delete_candidates": safe_deletes,
        "assets": rows,
    }


def _audit_markdown(audit: dict[str, Any]) -> str:
    lines = [
        "# Workcell Studio environment asset audit",
        "",
        f"Profile assets: **{audit['asset_count']}**  ",
        f"Known placeholders: **{audit['placeholder_count']}**  ",
        f"Awaiting downloaded models: **{len(audit['awaiting_model_asset_ids'])}**  ",
        f"Safe delete candidates: **{len(audit['safe_delete_candidates'])}**",
        "",
        "| Asset | Quality | Action | Status | Mesh | External references | Safe delete |",
        "|---|---|---|---|---|---:|---|",
    ]
    for row in audit["assets"]:
        lines.append(
            "| {asset_id} | {current_quality} | {action} | {replacement_status} | `{mesh_path}` | {refs} | {delete} |".format(
                **row,
                refs=len(row["references_outside_asset_package"]),
                delete="yes" if row["safe_to_delete_now"] else "no",
            )
        )
    if audit["missing_replacement_registry_entries"]:
        lines.extend(["", "## Missing registry entries", ""])
        lines.extend(f"- `{asset_id}`" for asset_id in audit["missing_replacement_registry_entries"])
    lines.extend(
        [
            "",
            "## Deletion policy",
            "",
            "A generated mesh is never deleted merely because it looks poor. It must be marked replaced or delete_when_unreferenced and have no references outside its own package.",
            "",
        ]
    )
    return "\n".join(lines)


def _write_output(text: str, output: str | None) -> None:
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        print(path)
    else:
        print(text)


def _safe_asset_id(value: str) -> str:
    value = value.strip().lower().replace("-", "_").replace(" ", "_")
    if not SAFE_ID.fullmatch(value):
        raise ValueError("asset id must match ^[a-z][a-z0-9_]*$")
    return value


def import_asset(args: argparse.Namespace) -> int:
    asset_id = _safe_asset_id(args.asset_id)
    visual_source = Path(args.visual_file).expanduser().resolve()
    license_source = Path(args.license_file).expanduser().resolve()
    if visual_source.suffix.lower() not in SUPPORTED_VISUAL_EXTENSIONS:
        raise ValueError(f"unsupported visual format: {visual_source.suffix}")
    if not visual_source.is_file():
        raise FileNotFoundError(visual_source)
    if not license_source.is_file():
        raise FileNotFoundError(license_source)
    if not args.redistribution_confirmed:
        raise ValueError("--redistribution-confirmed is required before copying a third-party model into the repository")
    dims = [float(value) for value in args.dimensions]
    if len(dims) != 3 or any(value <= 0 for value in dims):
        raise ValueError("--dimensions requires three positive metre values")

    package_name = f"{asset_id}_description"
    package = ROOT / "assets" / "environment" / package_name
    if package.exists():
        raise FileExistsError(f"asset package already exists: {_relative(package)}")
    visual_rel = Path("meshes") / "visual" / visual_source.name
    destination = package / visual_rel
    if args.dry_run:
        print(json.dumps({"package": _relative(package), "visual": str(visual_rel), "status": "dry_run"}, indent=2))
        return 0

    destination.parent.mkdir(parents=True, exist_ok=False)
    (package / "urdf").mkdir()
    shutil.copy2(visual_source, destination)
    shutil.copy2(license_source, package / "LICENSE.txt")

    x, y, z = dims
    (package / "package.xml").write_text(
        f'''<?xml version="1.0"?>\n<package format="3">\n  <name>{package_name}</name>\n  <version>0.1.0</version>\n  <description>Workcell Studio environment asset: {args.display_name}</description>\n  <maintainer email="maintainer@example.com">Workcell Studio</maintainer>\n  <license>{args.license_id}</license>\n  <buildtool_depend>ament_cmake</buildtool_depend>\n  <export><build_type>ament_cmake</build_type></export>\n</package>\n''',
        encoding="utf-8",
    )
    (package / "CMakeLists.txt").write_text(
        f'''cmake_minimum_required(VERSION 3.8)\nproject({package_name})\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY meshes urdf DESTINATION share/${{PROJECT_NAME}})\ninstall(FILES asset_manifest.yaml SOURCE.md LICENSE.txt DESTINATION share/${{PROJECT_NAME}})\nament_package()\n''',
        encoding="utf-8",
    )
    (package / "asset_manifest.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": "workcell_studio_environment_asset/v1",
                "asset": {
                    "id": asset_id,
                    "display_name": args.display_name,
                    "quality_tier": "candidate",
                    "approval_status": "awaiting_visual_review",
                    "visual_mesh": visual_rel.as_posix(),
                    "collision": {"type": "box", "dimensions_m": dims},
                    "dimensions_m": dims,
                    "source_url": args.source_url,
                    "author_or_manufacturer": args.author,
                    "license": args.license_id,
                    "redistribution_confirmed": True,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (package / "SOURCE.md").write_text(
        f'''# Source\n\n- Asset: {args.display_name}\n- Author/manufacturer: {args.author}\n- Source URL: {args.source_url}\n- Licence: {args.license_id}\n- Redistribution permission checked: yes\n- Original file: {visual_source.name}\n- Imported dimensions in metres: {x}, {y}, {z}\n- Import status: awaiting scale, origin, orientation and visual-quality review\n''',
        encoding="utf-8",
    )
    (package / "urdf" / f"{asset_id}.urdf.xacro").write_text(
        f'''<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{asset_id}">\n  <xacro:macro name="{asset_id}" params="prefix parent *origin">\n    <link name="${{prefix}}{asset_id}">\n      <visual>\n        <origin xyz="0 0 0" rpy="0 0 0"/>\n        <geometry><mesh filename="package://{package_name}/{visual_rel.as_posix()}" scale="1 1 1"/></geometry>\n      </visual>\n      <collision>\n        <origin xyz="0 0 {z / 2:.9g}" rpy="0 0 0"/>\n        <geometry><box size="{x:.9g} {y:.9g} {z:.9g}"/></geometry>\n      </collision>\n    </link>\n    <joint name="${{prefix}}{asset_id}_joint" type="fixed">\n      <parent link="${{parent}}"/>\n      <child link="${{prefix}}{asset_id}"/>\n      <xacro:insert_block name="origin"/>\n    </joint>\n  </xacro:macro>\n</robot>\n''',
        encoding="utf-8",
    )
    print(_relative(package))
    return 0


def prune_assets(apply: bool) -> int:
    audit = build_audit()
    candidates = [row for row in audit["assets"] if row["safe_to_delete_now"]]
    if not candidates:
        print("No safe placeholder mesh deletions. Referenced placeholders remain quarantined until their references are migrated.")
        return 0
    for row in candidates:
        path = ROOT / row["mesh_path"]
        if apply:
            path.unlink()
            print(f"deleted {_relative(path)}")
        else:
            print(f"would delete {_relative(path)}")
    if not apply:
        print("Dry run only; pass --apply to delete the listed unreferenced meshes.")
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit, safely prune, and import Workcell Studio environment assets.")
    sub = parser.add_subparsers(dest="command", required=True)

    audit = sub.add_parser("audit", help="Inspect the current environment asset profile and replacement registry.")
    audit.add_argument("--format", choices=("json", "markdown"), default="markdown")
    audit.add_argument("--output")

    prune = sub.add_parser("prune", help="Delete only registry-approved placeholder meshes with no remaining references.")
    prune.add_argument("--apply", action="store_true")

    imp = sub.add_parser("import", help="Create a reviewable ROS description package from a licensed visual model.")
    imp.add_argument("--asset-id", required=True)
    imp.add_argument("--display-name", required=True)
    imp.add_argument("--visual-file", required=True)
    imp.add_argument("--dimensions", nargs=3, metavar=("X", "Y", "Z"), required=True)
    imp.add_argument("--source-url", required=True)
    imp.add_argument("--author", required=True)
    imp.add_argument("--license-id", required=True)
    imp.add_argument("--license-file", required=True)
    imp.add_argument("--redistribution-confirmed", action="store_true")
    imp.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "audit":
            audit = build_audit()
            text = json.dumps(audit, indent=2, sort_keys=True) + "\n" if args.format == "json" else _audit_markdown(audit)
            _write_output(text, args.output)
            return 1 if audit["missing_replacement_registry_entries"] else 0
        if args.command == "prune":
            return prune_assets(args.apply)
        if args.command == "import":
            return import_asset(args)
    except (FileNotFoundError, FileExistsError, ValueError, json.JSONDecodeError, yaml.YAMLError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
