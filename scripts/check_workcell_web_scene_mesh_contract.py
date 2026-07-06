#!/usr/bin/env python3
"""Validate the staged mesh contract for a Workcell Studio web scene JSON.

The web exporter marks physical/core mesh visuals with ``mesh_load_required`` and,
when asset staging succeeds, rewrites them to browser-loadable relative URLs.  This
checker is intentionally filesystem-based so CI can catch a stale or incomplete
``build/workcell_studio_web_scene/*.web_scene.json`` bundle before the browser
viewer silently falls back to primitive boxes.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping
from urllib.parse import unquote, urlparse

Json = dict[str, Any]

_ITEM_SECTIONS = ("robots", "tools", "assets", "sensors", "zones")
_UNSAFE_URL_SCHEMES = {"", "http", "https"}
_PRIMITIVE_FALLBACK_TOKENS = (
    "primitive_fallback",
    "primitive fallback",
    "fallback primitive",
    "primitive-only",
    "primitive_only",
    "fallback_only",
    "fallback-only",
)


def _load_json(path: Path) -> Json:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"web scene JSON does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"web scene JSON root must be an object: {path}")
    return data


def _iter_items(payload: Mapping[str, Any]) -> Iterable[tuple[str, Json]]:
    for section in _ITEM_SECTIONS:
        values = payload.get(section) or []
        if not isinstance(values, list):
            continue
        for item in values:
            if isinstance(item, dict):
                yield section, item


def _item_label(section: str, item: Mapping[str, Any]) -> str:
    item_id = item.get("id") or item.get("display_name") or item.get("name") or "<unknown>"
    category = item.get("core_mesh_category") or item.get("category") or item.get("role")
    return f"{section}:{item_id}" + (f" ({category})" if category else "")


def _url_to_relative_path(url: Any) -> Path | None:
    if not isinstance(url, str) or not url.strip():
        return None
    raw = url.strip()
    parsed = urlparse(raw)
    if parsed.scheme not in _UNSAFE_URL_SCHEMES:
        return None
    if parsed.scheme in {"http", "https"} or parsed.netloc:
        return None
    path = unquote(parsed.path if parsed.scheme else raw)
    candidate = Path(path)
    if candidate.is_absolute() or any(part == ".." for part in candidate.parts):
        return None
    return candidate


def _resolve_staged_path(url: Any, web_scene_path: Path) -> Path | None:
    rel = _url_to_relative_path(url)
    if rel is None:
        return None

    output_dir = web_scene_path.parent.resolve()
    cwd = Path.cwd().resolve()
    candidates = ((cwd / rel).resolve(), (output_dir / rel).resolve())
    for candidate in candidates:
        try:
            candidate.relative_to(output_dir)
        except ValueError:
            continue
        if candidate.is_file():
            return candidate
    return None


def _has_browser_loadable_staged_url(item: Mapping[str, Any], web_scene_path: Path) -> bool:
    return _resolve_staged_path(item.get("mesh_url") or item.get("mesh_staged_path") or item.get("mesh_uri"), web_scene_path) is not None


def _is_primitive_fallback_only(item: Mapping[str, Any]) -> bool:
    text_parts: list[str] = []
    for key in (
        "source_layer",
        "active_visual_source",
        "geometry_type",
        "primitive_geometry_type",
        "mesh_staging_status",
        "status",
        "mesh_load_warning",
        "mesh_resolve_warning",
    ):
        value = item.get(key)
        if isinstance(value, list):
            text_parts.extend(str(v) for v in value)
        elif value is not None:
            text_parts.append(str(value))
    text = " ".join(text_parts).lower()
    has_mesh_url = bool(_url_to_relative_path(item.get("mesh_url") or item.get("mesh_staged_path") or item.get("mesh_uri")))
    explicit_fallback = any(token in text for token in _PRIMITIVE_FALLBACK_TOKENS)
    primitive_geometry = str(item.get("geometry_type") or item.get("primitive_geometry_type") or "").lower() in {"box", "cube", "sphere", "cylinder", "primitive"}
    return (explicit_fallback or primitive_geometry) and not has_mesh_url


def check(payload: Mapping[str, Any], web_scene_path: Path) -> tuple[Json, list[str]]:
    required_items = [(section, item) for section, item in _iter_items(payload) if item.get("mesh_load_required") is True]
    staged_items = [(section, item) for section, item in required_items if _has_browser_loadable_staged_url(item, web_scene_path)]
    missing = [(section, item) for section, item in required_items if not _has_browser_loadable_staged_url(item, web_scene_path)]
    primitive_fallback = [(section, item) for section, item in required_items if _is_primitive_fallback_only(item)]

    contract = payload.get("metadata", {}).get("mesh_contract", {}) if isinstance(payload.get("metadata"), dict) else {}
    contract_fallback_count = int(contract.get("fallback_primitive_count") or 0) if isinstance(contract, dict) else 0

    errors: list[str] = []
    if missing:
        errors.append("required mesh item(s) are missing browser-loadable staged files")
    if primitive_fallback or contract_fallback_count:
        errors.append("core mesh item(s) are primitive-fallback-only")

    summary: Json = {
        "required_mesh_count": len(required_items),
        "staged_mesh_count": len(staged_items),
        "missing_required_meshes": [_item_label(section, item) for section, item in missing],
        "primitive_fallback_only_core_meshes": [_item_label(section, item) for section, item in primitive_fallback],
        "metadata_fallback_primitive_count": contract_fallback_count,
        "contract_status": "passed" if not errors else "failed",
    }
    return summary, errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check staged Workcell Studio web scene mesh contract.")
    parser.add_argument("web_scene", type=Path, help="Staged *.web_scene.json, for example build/workcell_studio_web_scene/ur5_2f_test.web_scene.json")
    args = parser.parse_args(argv)

    try:
        payload = _load_json(args.web_scene)
        summary, errors = check(payload, args.web_scene)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"required_mesh_count: {summary['required_mesh_count']}")
    print(f"staged_mesh_count: {summary['staged_mesh_count']}")
    print(f"missing_required_meshes: {len(summary['missing_required_meshes'])}")
    for item in summary["missing_required_meshes"]:
        print(f"  - {item}")
    if summary["primitive_fallback_only_core_meshes"] or summary["metadata_fallback_primitive_count"]:
        print(f"primitive_fallback_only_core_meshes: {len(summary['primitive_fallback_only_core_meshes'])} (metadata fallback count: {summary['metadata_fallback_primitive_count']})")
        for item in summary["primitive_fallback_only_core_meshes"]:
            print(f"  - {item}")
    print(f"contract_status: {summary['contract_status']}")

    if errors:
        print("errors:")
        for error in errors:
            print(f"  - {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
