#!/usr/bin/env python3
"""Canonical-first saved-layout detection shared by acceptance utilities."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

CANONICAL_LAYOUT_REL = "layout/workcell_studio_layout.yaml"
LEGACY_LAYOUT_REL = "environment_layout.yaml"


def _load_mapping(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError):
        return None
    return payload if isinstance(payload, dict) else None


def inspect_saved_layout(scene_dir: Path) -> dict[str, Any]:
    canonical = scene_dir / CANONICAL_LAYOUT_REL
    canonical_payload = _load_mapping(canonical)
    if (
        canonical_payload is not None
        and canonical_payload.get("schema_version") == "workcell_studio_layout/v1"
        and isinstance(canonical_payload.get("items"), list)
    ):
        return {"saved": True, "source": "canonical", "path": canonical, "legacy_fallback": False}

    legacy = scene_dir / LEGACY_LAYOUT_REL
    legacy_payload = _load_mapping(legacy)
    if legacy_payload is not None and legacy_payload.get("schema_version") == "environment_layout/v1":
        return {"saved": True, "source": "legacy_fallback", "path": legacy, "legacy_fallback": True}

    if canonical.exists():
        blocker = "Invalid layout/workcell_studio_layout.yaml"
    elif legacy.exists():
        blocker = "Invalid legacy environment_layout.yaml"
    else:
        blocker = "Missing saved layout/workcell_studio_layout.yaml"
    return {"saved": False, "source": "none", "path": canonical, "legacy_fallback": False, "blocker": blocker}


def resolve_saved_layout_path(scene_dir: Path) -> Path | None:
    result = inspect_saved_layout(scene_dir)
    return result["path"] if result["saved"] else None
