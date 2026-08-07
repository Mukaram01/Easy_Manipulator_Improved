#!/usr/bin/env python3
"""Normalize Product View visual artifacts for portability and completeness."""

from __future__ import annotations

import re
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping, MutableMapping, Optional
from urllib.parse import unquote, urlparse

CONTRACT_SCHEMA = "workcell_studio_visual_artifact_portability/v1"
SCENE_SECTIONS = ("robots", "tools", "assets", "sensors", "zones", "frames")
MESH_REFERENCE_FIELDS = (
    "package_uri", "original_package_uri", "original_mesh_uri", "mesh_uri",
    "mesh_url", "mesh_staged_path", "source_path", "mesh_path", "filepath",
)
STAGED_ROOT = "build/workcell_studio_web_scene/assets/"
WINDOWS_ABSOLUTE = re.compile(r"^[A-Za-z]:[\\/]")


def _as_dict(value: Any) -> MutableMapping[str, Any]:
    return value if isinstance(value, MutableMapping) else {}


def _iter_items(payload: Mapping[str, Any]) -> Iterable[tuple[str, MutableMapping[str, Any]]]:
    for section in SCENE_SECTIONS:
        values = payload.get(section)
        if not isinstance(values, list):
            continue
        for value in values:
            if isinstance(value, MutableMapping):
                yield section, value


def _looks_like_repo_root(path: Path) -> bool:
    return all((path / name).exists() for name in ("assets", "scenes", "scripts"))


def _repo_root(scene_dir: Path, output_path: Optional[Path]) -> Path:
    candidates = [scene_dir.resolve(), *scene_dir.resolve().parents]
    if output_path is not None:
        output = Path(output_path).expanduser().resolve()
        candidates.extend((output.parent, *output.parents))
    cwd = Path.cwd().resolve()
    candidates.extend((cwd, *cwd.parents))
    seen: set[Path] = set()
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        if _looks_like_repo_root(candidate):
            return candidate
    return cwd


def _is_package_uri(value: Any) -> bool:
    return isinstance(value, str) and value.strip().startswith("package://")


def _is_remote_uri(value: str) -> bool:
    return urlparse(value).scheme.lower() in {"http", "https", "data", "blob", "qrc"}


def _is_absolute_path(value: Any) -> bool:
    if not isinstance(value, str) or not value.strip():
        return False
    text = value.strip()
    parsed = urlparse(text)
    if parsed.scheme == "file":
        return True
    if parsed.scheme and len(parsed.scheme) > 1:
        return False
    return text.startswith(("/", "\\")) or bool(WINDOWS_ABSOLUTE.match(text))


def _safe_relative(value: str) -> Optional[str]:
    text = value.strip().replace("\\", "/")
    if not text or _is_package_uri(text) or _is_remote_uri(text) or _is_absolute_path(text):
        return None
    while text.startswith("./"):
        text = text[2:]
    path = PurePosixPath(text)
    if not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        return None
    return path.as_posix()


def _portable_path(value: str, repo_root: Path, scene_dir: Path) -> Optional[str]:
    if _is_package_uri(value) or _is_remote_uri(value):
        return value.strip()
    relative = _safe_relative(value)
    if relative is not None:
        return relative
    if not _is_absolute_path(value):
        return None
    parsed = urlparse(value)
    raw_text = unquote(parsed.path) if parsed.scheme == "file" else value
    try:
        resolved = Path(raw_text).expanduser().resolve(strict=False)
    except OSError:
        return None
    for root in (repo_root.resolve(), scene_dir.resolve()):
        try:
            candidate = resolved.relative_to(root).as_posix()
        except ValueError:
            continue
        return candidate if candidate and candidate != "." else None
    return None


def _canonical_package_uri(item: Mapping[str, Any]) -> str:
    for field in ("package_uri", "original_package_uri", "original_mesh_uri", "mesh_uri", "source_path", "mesh_path", "filepath"):
        value = item.get(field)
        if _is_package_uri(value):
            return str(value).strip()
    return ""


def _canonical_staged_path(item: Mapping[str, Any]) -> str:
    for field in ("mesh_staged_path", "mesh_url", "mesh_uri"):
        value = item.get(field)
        if not isinstance(value, str):
            continue
        text = value.strip().replace("\\", "/").lstrip("/")
        if text.startswith(STAGED_ROOT) and _safe_relative(text):
            return text
    return ""


def _has_mesh_reference(item: Mapping[str, Any]) -> bool:
    return any(isinstance(item.get(field), str) and bool(str(item.get(field)).strip()) for field in MESH_REFERENCE_FIELDS)


def _has_physical_bounds(item: Mapping[str, Any]) -> bool:
    for field in ("dimensions", "size", "mesh_bounds", "local_bounds", "bounds"):
        value = item.get(field)
        if isinstance(value, (list, tuple)) and len(value) >= 3:
            return True
        if isinstance(value, Mapping) and value:
            return True
    geometry = str(item.get("geometry_type") or item.get("primitive_geometry_type") or "").strip().lower()
    return geometry in {"box", "sphere", "cylinder", "capsule"}


def _identity(item: Mapping[str, Any], section: str) -> str:
    fields = ("id", "type", "role", "category", "link", "object_name", "frame", "semantic_role", "source_layer", "active_visual_source")
    return " ".join(str(item.get(field) or "").lower().replace("-", "_") for field in fields) + " " + section.lower()


def _physical_role_identity(item: Mapping[str, Any], section: str) -> str:
    """Identity fields that are allowed to determine physical semantic type.

    Provenance/ownership fields such as source_layer and active_visual_source
    must not participate here: for example, "editable_authored_physical"
    contains the substring "table" inside "editable".
    """
    fields = (
        "id",
        "type",
        "role",
        "category",
        "link",
        "object_name",
        "frame",
        "semantic_role",
    )
    return (
        " ".join(
            str(item.get(field) or "").lower().replace("-", "_")
            for field in fields
        )
        + " "
        + section.lower()
    )


def _set_if_blank(item: MutableMapping[str, Any], field: str, value: str) -> None:
    if not str(item.get(field) or "").strip():
        item[field] = value


def _infer_physical_role(item: MutableMapping[str, Any], section: str, package_uri: str) -> None:
    identity = (_physical_role_identity(item, section) + " " + package_uri.lower()).replace("-", "_")
    if "ur_description/meshes/" in identity or section == "robots":
        role, category, semantic = "robot", "robot_static_mesh_visual", "robot_visual"
    elif "robotiq" in identity or "gripper" in identity or section == "tools":
        role, category, semantic = "gripper", "tool", "tool_visual"
    elif any(token in identity for token in ("realsense", "camera", "d435")) or section == "sensors":
        role, category, semantic = "camera", "camera", "configured_camera"
    elif any(token in identity for token in ("workbench", "support_surface", "table")):
        role, category, semantic = "support_surface", "table", "support_surface"
    else:
        role, category, semantic = "physical_asset", "asset", "physical_object"
    _set_if_blank(item, "role", role)
    _set_if_blank(item, "category", category)
    _set_if_blank(item, "semantic_role", semantic)


def _classify_non_mesh(item: MutableMapping[str, Any], section: str) -> tuple[str, str]:
    identity = _identity(item, section)
    if section == "zones" or str(item.get("render_policy") or "").lower() == "overlay":
        classification = "intentional_overlay"
        reason = str(item.get("render_policy_reason") or "task_or_diagnostic_overlay_is_not_a_mesh_visual")
        item.setdefault("render_policy", "overlay")
        item.setdefault("render_owner", "task_overlay")
    elif "transform_anchor" in identity or section == "frames" or "tool0" in identity:
        classification = "intentional_non_mesh_transform_anchor"
        reason = "meshless_transform_anchor_preserved_for_tf_and_tool_mount"
        item["type"] = "transform_anchor"
        item["role"] = "transform_anchor"
        _set_if_blank(item, "category", "frame")
        item["render_policy"] = "diagnostic_only"
        item["render_owner"] = "diagnostic_helper"
    else:
        classification = "intentional_nonvisual_semantic_record"
        reason = str(item.get("render_policy_reason") or "semantic_capability_record_has_no_independent_visual_geometry")
        _set_if_blank(item, "role", "metadata")
        _set_if_blank(item, "category", "metadata")
        item["render_policy"] = "diagnostic_only"
        item.setdefault("render_owner", "diagnostic_helper")
    item["visual_artifact_classification"] = classification
    item["intentional_exclusion_reason"] = reason
    if classification != "intentional_overlay":
        item["render_expected"] = False
        item["mesh_load_required"] = False
        item["selectable"] = False
        item["exclude_from_fit_bounds"] = True
    return classification, reason


def _sanitize_field(
    item: MutableMapping[str, Any], field: str, value: str,
    repo_root: Path, scene_dir: Path, package_uri: str, staged_path: str,
) -> tuple[bool, str]:
    if _is_package_uri(value) or _is_remote_uri(value):
        return False, ""
    portable = _portable_path(value, repo_root, scene_dir)
    if portable is not None:
        item[field] = portable
        return False, "rewritten_repository_relative" if portable != value else ""
    if not _is_absolute_path(value):
        return False, ""
    if package_uri and field not in {"mesh_uri", "mesh_url", "mesh_staged_path"}:
        item[field] = package_uri
        return True, "absolute_source_path_replaced_by_package_uri"
    if staged_path:
        item[field] = staged_path
        return True, "absolute_source_path_replaced_by_staged_path"
    item.pop(field, None)
    return True, "unportable_absolute_source_path_removed"


def _sanitize_item_paths(
    item: MutableMapping[str, Any], *, repo_root: Path, scene_dir: Path,
    package_uri: str, staged_path: str,
) -> int:
    removed = 0
    outcomes: list[str] = []

    resolved = item.get("resolved_source_path")
    if isinstance(resolved, str) and resolved.strip():
        portable = _portable_path(resolved, repo_root, scene_dir)
        if portable is not None and not _is_package_uri(portable):
            item["resolved_source_path"] = portable
            item["repo_relative_source_path"] = portable
            if portable != resolved:
                outcomes.append("resolved_source_path_rewritten_repository_relative")
        else:
            item.pop("resolved_source_path", None)
            removed += int(_is_absolute_path(resolved))
            outcomes.append("unportable_resolved_source_path_removed")
    original_resolved = item.pop("resolved_source_path_original", None)
    removed += int(_is_absolute_path(original_resolved))

    for field in ("original_source_path", "source_path", "mesh_path", "filepath", "original_mesh_uri", "mesh_uri", "mesh_url", "mesh_staged_path"):
        value = item.get(field)
        if not isinstance(value, str) or not value.strip():
            continue
        was_removed, outcome = _sanitize_field(item, field, value, repo_root, scene_dir, package_uri, staged_path)
        removed += int(was_removed)
        if outcome:
            outcomes.append(outcome)

    evidence = item.get("mesh_unit_evidence")
    if isinstance(evidence, MutableMapping):
        value = evidence.get("source_path")
        if isinstance(value, str) and value.strip() and not _is_package_uri(value):
            portable = _portable_path(value, repo_root, scene_dir)
            if portable is not None:
                evidence["source_path"] = portable
            elif _is_absolute_path(value):
                evidence.pop("source_path", None)
                removed += 1
                outcomes.append("diagnostic_absolute_source_path_removed")

    item["resolved_source_path_stale"] = False
    if outcomes:
        item["source_path_resolution_outcome"] = "+".join(dict.fromkeys(outcomes))
    elif package_uri and staged_path:
        item["source_path_resolution_outcome"] = "package_uri_authoritative_staged_repository_relative"
    elif package_uri:
        item["source_path_resolution_outcome"] = "package_uri_authoritative"
    elif staged_path:
        item["source_path_resolution_outcome"] = "staged_repository_relative"
    return removed


def normalize_web_scene_payload(
    payload: MutableMapping[str, Any], *, scene_dir: Path,
    output_path: Optional[Path] = None, stage_assets: bool = False,
) -> MutableMapping[str, Any]:
    """Normalize a web-scene payload in place and return it."""
    scene_dir = Path(scene_dir).expanduser().resolve()
    repo_root = _repo_root(scene_dir, output_path)
    package_count = staged_count = absolute_removed = total = 0
    intentional: list[dict[str, str]] = []

    for section, item in _iter_items(payload):
        total += 1
        package_uri = _canonical_package_uri(item)
        staged_path = _canonical_staged_path(item)
        if package_uri:
            package_count += 1
            item["package_uri"] = package_uri
            item["authoritative_mesh_reference"] = package_uri
            item["mesh_reference_authority"] = "package_uri"
        elif staged_path:
            item["authoritative_mesh_reference"] = staged_path
            item["mesh_reference_authority"] = "repository_relative_staged_path"
        if staged_path:
            staged_count += 1
            item["repo_relative_staged_path"] = staged_path
            item["mesh_uri"] = staged_path
            item["mesh_url"] = staged_path
            item["mesh_staged_path"] = staged_path

        absolute_removed += _sanitize_item_paths(
            item, repo_root=repo_root, scene_dir=scene_dir,
            package_uri=package_uri, staged_path=staged_path,
        )

        physical = _has_mesh_reference(item) or _has_physical_bounds(item)
        if physical:
            _infer_physical_role(item, section, package_uri)
            item["visual_artifact_classification"] = (
                "portable_mesh_visual_diagnostic"
                if str(item.get("render_policy") or "").lower() == "diagnostic_only"
                else "portable_physical_visual"
            )
            if str(item.get("render_policy") or "").lower() == "diagnostic_only":
                item.setdefault("intentional_exclusion_reason", str(item.get("render_policy_reason") or "authoritative_renderer_owns_equivalent_physical_visual"))
        else:
            classification, reason = _classify_non_mesh(item, section)
            intentional.append({
                "section": section,
                "id": str(item.get("id") or item.get("link") or item.get("frame") or "<unnamed>"),
                "classification": classification,
                "reason": reason,
            })

    stale: list[str] = []
    unknown: list[str] = []
    for section, item in _iter_items(payload):
        if item.get("resolved_source_path_stale") is True or _is_absolute_path(item.get("resolved_source_path")):
            stale.append(f"{section}:{item.get('id', '<unnamed>')}")
        if not item.get("visual_artifact_classification"):
            unknown.append(f"{section}:{item.get('id', '<unnamed>')}")

    summary = {
        "schema_version": CONTRACT_SCHEMA,
        "status": "PASS" if not stale and not unknown else "FAIL",
        "total_visual_artifact_records": total,
        "package_uri_authoritative_count": package_count,
        "repository_relative_staged_path_count": staged_count,
        "absolute_source_paths_removed": absolute_removed,
        "stale_resolved_source_path": len(stale),
        "stale_resolved_source_path_records": stale,
        "unknown_role_no_fallback": len(unknown),
        "unknown_role_no_fallback_records": unknown,
        "intentional_non_mesh_exclusion_count": len(intentional),
        "intentional_non_mesh_exclusions": intentional,
        "stage_assets_requested": bool(stage_assets),
    }
    metadata = _as_dict(payload.setdefault("metadata", {}))
    metadata["visual_artifact_portability"] = summary
    viewer_summary = payload.get("viewer_summary")
    if isinstance(viewer_summary, MutableMapping):
        viewer_summary.update({
            "visual_artifact_portability_status": summary["status"],
            "stale_resolved_source_path": summary["stale_resolved_source_path"],
            "unknown_role_no_fallback": summary["unknown_role_no_fallback"],
            "intentional_non_mesh_exclusion_count": summary["intentional_non_mesh_exclusion_count"],
        })
    return payload
