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


def test_scene3d_smoke_runner_is_thin_wrapper_wired_to_payload_helpers():
    wrapper_path = REPO_ROOT / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py"
    payload_helpers = _load_module("scene3d_smoke_payload_for_wrapper_test", REPO_ROOT / "scripts" / "scene3d_smoke_payload.py")
    wrapper = _load_module("scene3d_smoke_runner_wrapper_test", wrapper_path)

    assert wrapper._IMPL_PATH.name == "run_workcell_builder_scene3d_gui_smoke_impl.py"
    assert wrapper._IMPL._enforce_physical_render_evidence is payload_helpers.enforce_physical_render_evidence
    assert wrapper._IMPL._physical_rendered_count is payload_helpers.physical_rendered_count
    assert wrapper._enforce_physical_render_evidence is payload_helpers.enforce_physical_render_evidence
    assert wrapper.build_cmd is wrapper._IMPL.build_cmd

    payload = {"status": "PASS", "counters": {"mesh_rendered_count": 0}}
    result = wrapper._enforce_physical_render_evidence(payload)
    assert result["status"] == "FAIL"
    assert result["blockers"] == ["scene_rendered_no_physical_items"]


def test_scene3d_smoke_public_wrapper_no_longer_contains_legacy_blocker_bug():
    wrapper_text = (REPO_ROOT / "scripts" / "run_workcell_builder_scene3d_gui_smoke.py").read_text(encoding="utf-8")

    assert "run_workcell_builder_scene3d_gui_smoke_impl.py" in wrapper_text
    assert "blockers is not defined" not in wrapper_text
    assert "if \"scene_rendered_no_physical_items\" not in blockers" not in wrapper_text
    assert len(wrapper_text.splitlines()) < 100
