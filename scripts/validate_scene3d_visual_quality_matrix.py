#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml

SCHEMA = "workcell_studio_scene3d_visual_quality_matrix/v1"
SMOKE_SCHEMA = "workcell_studio_scene3d_gui_smoke/v1"
PHYSICAL_ITEM_DOMINANCE_RATIO = 0.5
PHYSICAL_FALLBACK_DOMINANCE_RATIO = 0.5

HELPER_OVERLAY_COUNTER_KEYS = (
    "overlay_helper_count",
    "overlay_count",
    "label_count",
    "warning_anchor_count",
    "fov_helper_count",
    "reach_helper_count",
    "safety_zone_count",
)
PHYSICAL_FALLBACK_COUNTER_KEYS = (
    "physical_fallback_count",
    "valid_physical_fallback_count",
    "primitive_fallback_count",
    "generated_fallback_count",
)
PHYSICAL_FIT_BOUNDS_COUNTER_KEYS = (
    "physical_fit_bounds_count",
    "physical_fit_bound_count",
    "physical_fit_bounds_item_count",
    "physical_items_in_fit_bounds_count",
    "physical_fit_included_count",
)
HELPER_FIT_BOUNDS_COUNTER_KEYS = (
    "helper_fit_bounds_count",
    "overlay_fit_bounds_count",
    "overlay_helper_fit_bounds_count",
    "helper_items_in_fit_bounds_count",
    "overlay_items_in_fit_bounds_count",
    "overlay_fit_included_count",
)

MESH_KEYS = {"mesh", "mesh_path", "mesh_uri", "mesh_filename", "filename", "resource", "uri"}
PRIMITIVE_TYPES = {"box", "cube", "cylinder", "sphere", "capsule", "cone", "primitive"}


def _load_json(path: Path) -> dict[str, Any]:
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_load_error": str(exc)}
    return loaded if isinstance(loaded, dict) else {"_load_error": "JSON root is not an object"}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_load_error": str(exc)}
    return loaded if isinstance(loaded, dict) else {"_load_error": "YAML root is not an object"}


def _as_int(value: Any, default: int = 0) -> int:
    try:
        if value is None or isinstance(value, bool):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _counter_present(payload: dict[str, Any], *keys: str) -> bool:
    sources = [payload]
    for nested in ("counters", "render_debug_counters"):
        nested_payload = payload.get(nested)
        if isinstance(nested_payload, dict):
            sources.append(nested_payload)
    return any(key in source for source in sources for key in keys)


def _counter(payload: dict[str, Any], *keys: str) -> int:
    sources = [payload]
    for nested in ("counters", "render_debug_counters"):
        nested_payload = payload.get(nested)
        if isinstance(nested_payload, dict):
            sources.append(nested_payload)
    for source in sources:
        for key in keys:
            if key in source:
                return _as_int(source.get(key))
    return 0


def _list_field(payload: dict[str, Any], *keys: str) -> list[Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, list):
            return value
    return []


def _stringify(value: Any) -> str:
    return str(value or "").strip().lower()


def _looks_like_mesh(item: dict[str, Any]) -> bool:
    for key in MESH_KEYS:
        value = item.get(key)
        if isinstance(value, str) and value.strip():
            if key == "mesh" and value.lower() in PRIMITIVE_TYPES:
                continue
            return True
        if isinstance(value, dict):
            filename = value.get("filename") or value.get("uri") or value.get("path")
            if isinstance(filename, str) and filename.strip():
                return True
    geometry = item.get("geometry")
    if isinstance(geometry, dict):
        if _looks_like_mesh(geometry):
            return True
        nested_mesh = geometry.get("mesh")
        if isinstance(nested_mesh, dict):
            return _looks_like_mesh(nested_mesh)
        if isinstance(nested_mesh, str) and nested_mesh.strip() and nested_mesh.lower() not in PRIMITIVE_TYPES:
            return True
    geometry_type = _stringify(item.get("geometry_type") or item.get("visual_type") or item.get("type"))
    return geometry_type == "mesh"


def _looks_like_primitive(item: dict[str, Any]) -> bool:
    for key in ("primitive_type", "shape", "geometry_type", "visual_type", "type"):
        if _stringify(item.get(key)) in PRIMITIVE_TYPES:
            return True
    geometry = item.get("geometry")
    if isinstance(geometry, str):
        return _stringify(geometry) in PRIMITIVE_TYPES
    if isinstance(geometry, dict):
        for primitive in PRIMITIVE_TYPES:
            if primitive in geometry and primitive != "mesh":
                return True
        return _looks_like_primitive(geometry)
    for key in ("box", "cylinder", "sphere", "capsule", "cone"):
        if key in item:
            return True
    return False


def _visual_items(mesh_index: dict[str, Any]) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for raw in [*_list_field(mesh_index, "visual_items"), *_list_field(mesh_index, "items")]:
        if isinstance(raw, dict):
            items.append(raw)
    return items


def classify_source_geometry(mesh_index: dict[str, Any]) -> tuple[int, int, int, int]:
    """Return total payload, mesh source, primitive source, missing geometry counts."""
    items = _visual_items(mesh_index)
    mesh_source_count = 0
    primitive_source_count = 0
    missing_geometry_count = 0
    for item in items:
        has_mesh = _looks_like_mesh(item)
        has_primitive = _looks_like_primitive(item)
        if has_mesh:
            mesh_source_count += 1
        elif has_primitive:
            primitive_source_count += 1
        else:
            missing_geometry_count += 1

    if not items:
        mesh_source_count = max(
            _as_int(mesh_index.get("mesh_source_count")),
            _as_int(mesh_index.get("candidate_mesh_count")),
            _as_int(mesh_index.get("renderable_mesh_count")),
        )
        primitive_source_count = max(
            _as_int(mesh_index.get("primitive_source_count")),
            _as_int(mesh_index.get("primitive_visual_count")),
        )
        missing_geometry_count = _as_int(mesh_index.get("unresolved_placeholder_count"))

    total_payload_count = max(
        len(items),
        _as_int(mesh_index.get("total_payload_count")),
        _as_int(mesh_index.get("visual_count")),
        _as_int(mesh_index.get("renderable_item_count")),
        mesh_source_count + primitive_source_count + missing_geometry_count,
    )
    if missing_geometry_count == 0 and total_payload_count > mesh_source_count + primitive_source_count:
        missing_geometry_count = total_payload_count - mesh_source_count - primitive_source_count
    return total_payload_count, mesh_source_count, primitive_source_count, missing_geometry_count


def _scene_entries(catalog_path: Path) -> list[dict[str, Any]]:
    catalog = _load_yaml(catalog_path)
    scenes = catalog.get("scenes")
    if not isinstance(scenes, list):
        return []
    entries: list[dict[str, Any]] = []
    for entry in scenes:
        if not isinstance(entry, dict):
            continue
        enabled = bool(entry.get("enabled", True))
        status = _stringify(entry.get("status"))
        support_level = _stringify(entry.get("support_level"))
        if enabled and status not in {"disabled", "ignored"} and support_level not in {"disabled", "ignored"}:
            entries.append(entry)
    return entries


def _resolve_mesh_index(scene_dir: Path) -> Path:
    if scene_dir.is_file():
        return scene_dir
    return scene_dir / "generated" / "scene_visual_mesh_index.json"


def _resolve_smoke_json(smoke_dir: Path | None, scene_name: str, scene_dir: Path) -> Path | None:
    candidates: list[Path] = []
    if smoke_dir is not None:
        candidates.extend([
            smoke_dir / f"scene3d_gui_smoke_{scene_name}.json",
            smoke_dir / f"{scene_name}.json",
        ])
    candidates.append(scene_dir / "generated" / "scene3d_gui_smoke.json")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0] if candidates else None


def _resolve_screenshot(screenshot_dir: Path | None, scene_name: str) -> Path | None:
    if screenshot_dir is None:
        return None
    for suffix in ("png", "jpg", "jpeg"):
        candidate = screenshot_dir / f"scene3d_gui_smoke_{scene_name}.{suffix}"
        if candidate.exists():
            return candidate
        candidate = screenshot_dir / f"{scene_name}.{suffix}"
        if candidate.exists():
            return candidate
    return screenshot_dir / f"scene3d_gui_smoke_{scene_name}.png"


def evaluate_scene(
    *,
    scene_name: str,
    scene_dir: Path,
    mesh_index_path: Path,
    smoke_json_path: Path | None,
    screenshot_path: Path | None = None,
    synthetic_fixture: bool = False,
) -> dict[str, Any]:
    warnings: list[str] = []
    blockers: list[str] = []
    blocker_reasons: list[str] = []

    def add_blocker(message: str, reason: str | None = None) -> None:
        blockers.append(message)
        if reason and reason not in blocker_reasons:
            blocker_reasons.append(reason)

    mesh_index: dict[str, Any] = {}
    if not mesh_index_path.exists():
        blockers.append(f"missing mesh/source payload: {mesh_index_path}")
    else:
        mesh_index = _load_json(mesh_index_path)
        if mesh_index.get("_load_error"):
            blockers.append(f"could not read mesh/source payload: {mesh_index_path}: {mesh_index['_load_error']}")

    total_payload_count, mesh_source_count, primitive_source_count, missing_geometry_count = classify_source_geometry(mesh_index)

    smoke_payload: dict[str, Any] = {}
    smoke_exists = bool(smoke_json_path and smoke_json_path.exists())
    if smoke_exists and smoke_json_path is not None:
        smoke_payload = _load_json(smoke_json_path)
        if smoke_payload.get("_load_error"):
            add_blocker(
                f"smoke_json_unreadable: could not read smoke JSON: {smoke_json_path}: {smoke_payload['_load_error']}",
                "smoke_json_unreadable",
            )
        elif smoke_payload.get("schema") not in {SMOKE_SCHEMA, None}:
            warnings.append(f"unexpected smoke schema: {smoke_payload.get('schema')!r}")
    elif smoke_json_path is not None:
        add_blocker(f"smoke_json_missing: smoke JSON not found: {smoke_json_path}", "smoke_json_missing")
    else:
        warnings.append("smoke JSON not provided")

    mesh_rendered_count = _counter(smoke_payload, "mesh_rendered_count", "mesh_backed_count")
    primitive_rendered_count = _counter(smoke_payload, "primitive_rendered_count", "urdf_primitive_rendered_count")
    placeholder_count = _counter(smoke_payload, "placeholder_count", "placeholder_box_count", "primitive_fallback_count")
    wireframe_fallback_count = _counter(smoke_payload, "wireframe_fallback_count", "wireframe_box_count")
    rendered_count = _counter(smoke_payload, "rendered_count")

    helper_overlay_counts = {key: _counter(smoke_payload, key) for key in HELPER_OVERLAY_COUNTER_KEYS}
    helper_overlay_count = max(helper_overlay_counts["overlay_helper_count"], helper_overlay_counts["overlay_count"]) + sum(
        helper_overlay_counts[key] for key in ("label_count", "warning_anchor_count", "fov_helper_count", "reach_helper_count", "safety_zone_count")
    )
    physical_fallback_raw_count = max(_counter(smoke_payload, key) for key in PHYSICAL_FALLBACK_COUNTER_KEYS)
    physical_rendered_base_count = mesh_rendered_count + primitive_rendered_count
    physical_fallback_count = 0
    if physical_fallback_raw_count > 0:
        fallback_limit = max(1, int(physical_rendered_base_count * PHYSICAL_FALLBACK_DOMINANCE_RATIO))
        if physical_rendered_base_count > 0 and physical_fallback_raw_count <= fallback_limit:
            physical_fallback_count = physical_fallback_raw_count
    physical_rendered_count = physical_rendered_base_count + physical_fallback_count

    physical_fit_bounds_present = _counter_present(smoke_payload, *PHYSICAL_FIT_BOUNDS_COUNTER_KEYS)
    helper_fit_bounds_present = _counter_present(smoke_payload, *HELPER_FIT_BOUNDS_COUNTER_KEYS)
    physical_fit_bounds_count = max(_counter(smoke_payload, key) for key in PHYSICAL_FIT_BOUNDS_COUNTER_KEYS)
    helper_fit_bounds_count = max(_counter(smoke_payload, key) for key in HELPER_FIT_BOUNDS_COUNTER_KEYS)

    if mesh_source_count > 0 and mesh_rendered_count <= 0:
        blockers.append("mesh_source_count > 0 requires mesh_rendered_count > 0")
    if primitive_source_count > 0 and primitive_rendered_count <= 0:
        blockers.append("primitive_source_count > 0 requires primitive_rendered_count > 0")
    if primitive_source_count > 0 and placeholder_count > missing_geometry_count:
        blockers.append("valid URDF primitives must not increment placeholder_count")
    if total_payload_count > 0 and mesh_source_count + primitive_source_count <= 0:
        blockers.append("source geometry classification missing; rendered_count alone cannot prove visual quality")
    if rendered_count == total_payload_count and total_payload_count > 0:
        warnings.append("rendered_count equals total_payload_count; pass still requires mesh/primitive-specific rendered counts")

    visible_physical_count = physical_rendered_count + placeholder_count + wireframe_fallback_count
    if wireframe_fallback_count > 0 and visible_physical_count > 0:
        if wireframe_fallback_count > int(visible_physical_count * PHYSICAL_ITEM_DOMINANCE_RATIO):
            blockers.append("wireframe_fallback_count dominates visible physical items")
    if helper_overlay_count > 0 and helper_overlay_count >= physical_rendered_count:
        add_blocker(
            "helper/overlay render counters dominate physical rendered evidence "
            f"(helper_overlay_count={helper_overlay_count}, physical_rendered_count={physical_rendered_count})",
            "overlay_helper_domination",
        )
    if helper_overlay_count > 0 and physical_rendered_count <= 0 and rendered_count > 0:
        add_blocker(
            "helper/overlay counters are the only render evidence; rendered_count alone cannot prove physical Scene3D quality",
            "overlay_only_render_evidence",
        )
    if physical_fit_bounds_present and helper_fit_bounds_present and helper_fit_bounds_count > 0 and helper_fit_bounds_count >= physical_fit_bounds_count:
        add_blocker(
            "helper/overlay fit-bounds counters dominate physical fit-bounds counters "
            f"(helper_fit_bounds_count={helper_fit_bounds_count}, physical_fit_bounds_count={physical_fit_bounds_count})",
            "overlay_fit_bounds_domination",
        )
    if missing_geometry_count > 0:
        warnings.append(f"{missing_geometry_count} source payload item(s) lack mesh or primitive geometry")

    if screenshot_path is not None and not screenshot_path.exists():
        add_blocker(f"screenshot_missing: screenshot not found: {screenshot_path}", "screenshot_missing")

    if synthetic_fixture:
        if mesh_source_count < 1 or primitive_source_count < 1:
            blockers.append("synthetic fixture must contain at least one mesh and one primitive source")
        if mesh_rendered_count < 1 or primitive_rendered_count < 1:
            blockers.append("synthetic fixture must render both one mesh and one primitive")
        if placeholder_count != 0:
            blockers.append("synthetic fixture must render mesh and primitive without placeholder boxes")

    visual_quality_status = "PASS" if not blockers else "FAIL"
    return {
        "scene_name": scene_name,
        "scene_path": str(scene_dir),
        "mesh_index_path": str(mesh_index_path),
        "smoke_json": str(smoke_json_path) if smoke_json_path is not None else "",
        "screenshot": str(screenshot_path) if screenshot_path is not None else "",
        "total_payload_count": total_payload_count,
        "mesh_source_count": mesh_source_count,
        "mesh_rendered_count": mesh_rendered_count,
        "primitive_source_count": primitive_source_count,
        "primitive_rendered_count": primitive_rendered_count,
        "placeholder_count": placeholder_count,
        "missing_geometry_count": missing_geometry_count,
        "wireframe_fallback_count": wireframe_fallback_count,
        "rendered_count": rendered_count,
        "physical_rendered_count": physical_rendered_count,
        "physical_fallback_count": physical_fallback_count,
        "physical_fallback_raw_count": physical_fallback_raw_count,
        "helper_overlay_count": helper_overlay_count,
        **helper_overlay_counts,
        "physical_fit_bounds_count": physical_fit_bounds_count,
        "helper_fit_bounds_count": helper_fit_bounds_count,
        "visual_quality_status": visual_quality_status,
        "warnings": warnings,
        "blockers": blockers,
        "blocker_reasons": blocker_reasons,
    }


def build_matrix(
    *,
    repo_root: Path,
    supported_scenes: Path,
    synthetic_fixture: Path,
    smoke_dir: Path | None = None,
    screenshot_dir: Path | None = None,
) -> dict[str, Any]:
    scenes: list[dict[str, Any]] = []
    for entry in _scene_entries(supported_scenes):
        scene_name = str(entry.get("scene_name") or entry.get("package_name") or "").strip()
        if not scene_name:
            continue
        scene_dir = repo_root / str(entry.get("scene_path") or f"scenes/{scene_name}")
        mesh_index_path = _resolve_mesh_index(scene_dir)
        scenes.append(
            evaluate_scene(
                scene_name=scene_name,
                scene_dir=scene_dir,
                mesh_index_path=mesh_index_path,
                smoke_json_path=_resolve_smoke_json(smoke_dir, scene_name, scene_dir),
                screenshot_path=_resolve_screenshot(screenshot_dir, scene_name),
            )
        )

    synthetic_path = synthetic_fixture
    synthetic_scene_dir = synthetic_path if synthetic_path.is_dir() else synthetic_path.parent
    synthetic_name = synthetic_scene_dir.name or "synthetic_visual_quality_fixture"
    synthetic_smoke = _resolve_smoke_json(smoke_dir, synthetic_name, synthetic_scene_dir)
    if synthetic_path.is_file() and synthetic_path.name.endswith(".json"):
        mesh_index_path = synthetic_path
        if synthetic_path.name.startswith("scene3d_gui_smoke") or "smoke" in synthetic_path.name:
            synthetic_smoke = synthetic_path
            mesh_index_path = synthetic_scene_dir / "scene_visual_mesh_index.json"
    else:
        mesh_index_path = _resolve_mesh_index(synthetic_scene_dir)
    scenes.append(
        evaluate_scene(
            scene_name=synthetic_name,
            scene_dir=synthetic_scene_dir,
            mesh_index_path=mesh_index_path,
            smoke_json_path=synthetic_smoke,
            screenshot_path=_resolve_screenshot(screenshot_dir, synthetic_name),
            synthetic_fixture=True,
        )
    )

    blockers = [f"{scene['scene_name']}: {blocker}" for scene in scenes for blocker in scene.get("blockers", [])]
    return {
        "schema": SCHEMA,
        "repo_root": str(repo_root),
        "supported_scenes": str(supported_scenes),
        "synthetic_fixture": str(synthetic_fixture),
        "scene_count": len(scenes),
        "scenes": scenes,
        "blockers": blockers,
        "pass": not blockers,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate Scene3D per-scene mesh/primitive visual quality evidence.")
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--supported-scenes", type=Path, default=None, help="Catalog YAML, defaults to <repo>/scenes/supported_scenes.yaml")
    parser.add_argument("--synthetic-fixture", type=Path, required=True, help="Synthetic generated scene directory or mesh-index JSON with one mesh and one primitive")
    parser.add_argument("--smoke-dir", type=Path, default=None, help="Optional directory containing scene3d_gui_smoke_<scene>.json files")
    parser.add_argument("--screenshot-dir", type=Path, default=None, help="Optional directory containing scene3d_gui_smoke_<scene>.png files")
    parser.add_argument("--json", type=Path, default=None, help="Output JSON path; defaults to stdout only")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = args.repo_root.resolve()
    supported_scenes = (args.supported_scenes or (repo_root / "scenes" / "supported_scenes.yaml")).resolve()
    payload = build_matrix(
        repo_root=repo_root,
        supported_scenes=supported_scenes,
        synthetic_fixture=args.synthetic_fixture.resolve(),
        smoke_dir=args.smoke_dir.resolve() if args.smoke_dir else None,
        screenshot_dir=args.screenshot_dir.resolve() if args.screenshot_dir else None,
    )
    rendered = json.dumps(payload, indent=2)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if payload["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
