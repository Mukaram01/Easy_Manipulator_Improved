#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


payload_helpers = _load_module("scene3d_smoke_payload", REPO_ROOT / "scripts" / "scene3d_smoke_payload.py")


def test_enforce_physical_render_evidence_initializes_missing_blockers():
    payload = {"status": "PASS", "counters": {"mesh_rendered_count": 0}}

    result = payload_helpers.enforce_physical_render_evidence(payload)

    assert result["runtime_available"] is True
    assert result["status"] == "FAIL"
    assert result["physical_rendered_count"] == 0
    assert result["blockers"] == ["scene_rendered_no_physical_items"]


def test_enforce_physical_render_evidence_preserves_existing_blockers_without_duplicates():
    payload = {
        "blockers": ["existing_issue", "scene_rendered_no_physical_items"],
        "render_debug_counters": {"primitive_rendered_count": 0},
    }

    result = payload_helpers.enforce_physical_render_evidence(payload)

    assert result["blockers"] == ["existing_issue", "scene_rendered_no_physical_items"]


def test_enforce_physical_render_evidence_passes_through_when_physical_items_rendered():
    payload = {"status": "PASS", "counters": {"mesh_rendered_count": 2}}

    result = payload_helpers.enforce_physical_render_evidence(payload)

    assert result["status"] == "PASS"
    assert "blockers" not in result
    assert result["runtime_available"] is True


def test_physical_rendered_count_accepts_nested_aliases():
    assert payload_helpers.physical_rendered_count({"counters": {"primitive_fallback_items_rendered": 3}}) == 3
    assert payload_helpers.physical_rendered_count({"render_debug_counters": {"mesh_backed_count": 4}}) == 4
    assert payload_helpers.physical_rendered_count({"credible_physical_rendered_count": 5}) == 5
