#!/usr/bin/env python3
"""Compatibility entry point for the Workcell Studio scene readiness matrix.

The established implementation lives in
``run_workcell_studio_scene_readiness_matrix_impl.py``.  It is executed into this
module's namespace so existing imports, monkeypatches, CLI callers, and tests keep
their public contract while policy-only follow-ups remain small and reviewable.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

_IMPL_PATH = Path(__file__).with_name("run_workcell_studio_scene_readiness_matrix_impl.py")
_PUBLIC_MODULE_NAME = __name__

# Execute the unchanged implementation into this module namespace.  Temporarily
# use a non-main module name so the implementation's CLI guard does not run
# before the compatibility policy below is installed.
_impl_source = _IMPL_PATH.read_text(encoding="utf-8")
globals()["__name__"] = "scripts.run_workcell_studio_scene_readiness_matrix_impl_exec"
exec(compile(_impl_source, str(_IMPL_PATH), "exec"), globals(), globals())
globals()["__name__"] = _PUBLIC_MODULE_NAME

_ORIGINAL_CHECK_MANIFEST_REFS = _check_manifest_refs
_ORIGINAL_CHECK_SCENE3D = _check_scene3d
_OPTIONAL_SCENE3D_MANIFEST_FIELDS = {
    "files.scene3d_gui_smoke",
}


def _is_optional_missing_scene3d_reference(item: Any) -> bool:
    if not isinstance(item, dict):
        return False
    field = str(item.get("field") or "")
    reason = str(item.get("reason") or "")
    return (
        field in _OPTIONAL_SCENE3D_MANIFEST_FIELDS
        and reason == "referenced file does not exist"
    )


def _check_manifest_refs(scene_dir: Path) -> dict[str, Any]:
    """Do not make absent optional Scene3D debug evidence a manifest failure."""
    result = _ORIGINAL_CHECK_MANIFEST_REFS(scene_dir)
    if str(result.get("status") or "").upper() != FAIL:
        return result

    missing = result.get("missing")
    if not isinstance(missing, list) or not missing:
        return result
    optional_missing = [item for item in missing if _is_optional_missing_scene3d_reference(item)]
    required_missing = [item for item in missing if not _is_optional_missing_scene3d_reference(item)]
    if not optional_missing or required_missing:
        return result

    checked = result.get("checked") if isinstance(result.get("checked"), list) else []
    required_checked = [
        item
        for item in checked
        if not (
            isinstance(item, dict)
            and str(item.get("field") or "") in _OPTIONAL_SCENE3D_MANIFEST_FIELDS
        )
    ]
    return _result(
        PASS,
        f"manifest local-file references resolved ({len(required_checked)} required checked); optional Scene3D debug evidence is absent",
        checked=required_checked,
        optional_debug_references=optional_missing,
        optional_scene3d_debug_evidence=True,
    )


def _optional_scene3d_debug_result(result: dict[str, Any], label: str) -> dict[str, Any]:
    """Preserve Scene3D diagnostics without letting them gate production readiness."""
    normalized = dict(result)
    debug_status = str(normalized.get("status") or "UNKNOWN").upper()
    normalized["debug_status"] = debug_status
    normalized["optional_debug_evidence"] = True
    normalized["production_readiness_gate"] = False
    if debug_status == PASS:
        return normalized

    diagnostic_blockers = normalized.get("blockers")
    if not isinstance(diagnostic_blockers, list):
        diagnostic_blockers = []
    normalized["diagnostic_blockers"] = list(diagnostic_blockers)
    normalized["blockers"] = []
    normalized["status"] = PASS
    normalized["message"] = (
        f"optional Scene3D {label} evidence is {debug_status}; recorded for diagnostics only and not used as a production-readiness gate"
    )
    return normalized


def _check_scene3d(scene_name: str, scene_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    visual_result, physical_result = _ORIGINAL_CHECK_SCENE3D(scene_name, scene_dir)
    return (
        _optional_scene3d_debug_result(visual_result, "visual-quality"),
        _optional_scene3d_debug_result(physical_result, "physical-render"),
    )


if _PUBLIC_MODULE_NAME == "__main__":
    raise SystemExit(main())
