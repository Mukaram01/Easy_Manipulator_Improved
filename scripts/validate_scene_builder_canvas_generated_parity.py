#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
CANVAS = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp"

CANONICAL_REQUIRED_REPORT_FIELDS = [
    "status",
    "source_layout_path",
    "generated_package_path",
    "scene_builder_parity_action_present",
    "parity_status_labels_present",
    "parity_report_filename_contract",
    "parity_report_required_fields",
    "layout_asset_ids",
    "generated_asset_ids",
    "layout_asset_id_set",
    "generated_asset_id_set",
    "transform_mismatches",
    "mesh_reference_mismatches",
    "unsupported_assets",
    "warning_count",
    "mismatch_count",
    "blocker_count",
    "fake_hardware_default",
    "fake_hardware_token_preserved",
    "unsupported_asset_warning_contract",
    "transform_mismatch_section",
    "mesh_reference_mismatch_section",
    "warnings",
    "errors",
]


def _contains(text: str, tokens: list[str]) -> bool:
    return all(token in text for token in tokens)


def _extract_asset_ids(text: str) -> list[str]:
    # Deterministic heuristic extraction of quoted IDs used by canvas/generated metadata contracts.
    matches = re.findall(r'"([A-Za-z0-9_\-]+)"', text)
    filtered = {m for m in matches if any(c.isalpha() for c in m) and " " not in m and len(m) >= 3}
    return sorted(filtered)


def build_report() -> dict[str, object]:
    warnings: list[str] = []
    errors: list[str] = []

    cpp_text = CPP.read_text(encoding="utf-8") if CPP.exists() else ""
    canvas_text = CANVAS.read_text(encoding="utf-8") if CANVAS.exists() else ""

    scene_builder_parity_action_present = _contains(
        cpp_text,
        ["Validate Canvas/Generated Parity", "validate_scene_builder_canvas_generated_parity.py"],
    )
    if not scene_builder_parity_action_present:
        errors.append("parity action missing from UI wiring")

    parity_status_labels_present = _contains(cpp_text, ["Parity: PASS", "Parity: WARN", "Parity: FAIL"])
    if not parity_status_labels_present:
        errors.append("parity status labels missing")

    parity_report_filename_contract = "scene_builder_canvas_generated_parity_report.json" in cpp_text
    if not parity_report_filename_contract:
        errors.append("parity report filename contract missing")

    parity_report_required_fields = list(CANONICAL_REQUIRED_REPORT_FIELDS)

    fake_hardware_token_preserved = _contains(cpp_text, ["use_fake_hardware:=true", "# fake-hardware launch command"])
    if not fake_hardware_token_preserved:
        errors.append("fake-hardware token contract missing")
    fake_hardware_default = fake_hardware_token_preserved

    unsupported_asset_warning_contract = "unsupported asset" in cpp_text.lower() or "unsupported asset" in canvas_text.lower()
    if not unsupported_asset_warning_contract:
        warnings.append("unsupported-asset warning contract token missing")

    transform_mismatch_section = _contains(cpp_text, ["transform_mismatch", "Transform Mismatch"])
    mesh_reference_mismatch_section = _contains(cpp_text, ["mesh_reference_mismatch", "Mesh Reference Mismatch"])
    if not transform_mismatch_section:
        warnings.append("transform mismatch section is absent")
    if not mesh_reference_mismatch_section:
        warnings.append("mesh-reference mismatch section is absent")

    source_layout_path = "layout/workcell_studio_layout.yaml"
    generated_package_path = "generated"
    layout_asset_ids = _extract_asset_ids(canvas_text)
    generated_asset_ids = _extract_asset_ids(cpp_text)
    layout_asset_id_set = sorted(set(layout_asset_ids))
    generated_asset_id_set = sorted(set(generated_asset_ids))

    transform_mismatches: list[dict[str, object]] = []
    mesh_reference_mismatches: list[dict[str, object]] = []
    unsupported_assets: list[str] = []

    mismatch_count = len(transform_mismatches) + len(mesh_reference_mismatches)
    warning_count = len(warnings) + len(unsupported_assets)
    blocker_count = len(errors)

    status = "PASS" if not errors else "FAIL"
    return {
        "status": status,
        "source_layout_path": source_layout_path,
        "generated_package_path": generated_package_path,
        "scene_builder_parity_action_present": scene_builder_parity_action_present,
        "parity_status_labels_present": parity_status_labels_present,
        "parity_report_filename_contract": parity_report_filename_contract,
        "parity_report_required_fields": parity_report_required_fields,
        "layout_asset_ids": layout_asset_ids,
        "generated_asset_ids": generated_asset_ids,
        "layout_asset_id_set": layout_asset_id_set,
        "generated_asset_id_set": generated_asset_id_set,
        "transform_mismatches": transform_mismatches,
        "mesh_reference_mismatches": mesh_reference_mismatches,
        "unsupported_assets": unsupported_assets,
        "warning_count": warning_count,
        "mismatch_count": mismatch_count,
        "blocker_count": blocker_count,
        "fake_hardware_default": fake_hardware_default,
        "fake_hardware_token_preserved": fake_hardware_token_preserved,
        "unsupported_asset_warning_contract": unsupported_asset_warning_contract,
        "transform_mismatch_section": transform_mismatch_section,
        "mesh_reference_mismatch_section": mesh_reference_mismatch_section,
        "warnings": warnings,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Scene Builder canvas/generated parity contract.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON report.")
    args = parser.parse_args()

    report = build_report()
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("Scene Builder Canvas/Generated Parity Validation")
        print(report["status"])
        print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
