#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
CANVAS = ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp"

REQUIRED_REPORT_FIELDS = [
    "status",
    "scene_builder_parity_action_present",
    "parity_status_labels_present",
    "parity_report_filename_contract",
    "parity_report_required_fields",
    "fake_hardware_token_preserved",
    "unsupported_asset_warning_contract",
    "transform_mismatch_section",
    "mesh_reference_mismatch_section",
    "warnings",
    "errors",
]


def _contains(text: str, tokens: list[str]) -> bool:
    return all(token in text for token in tokens)


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

    parity_report_required_fields = list(REQUIRED_REPORT_FIELDS)

    fake_hardware_token_preserved = _contains(cpp_text, ["use_fake_hardware:=true", "# fake-hardware launch command"])
    if not fake_hardware_token_preserved:
        errors.append("fake-hardware token contract missing")

    unsupported_asset_warning_contract = "unsupported asset" in cpp_text.lower() or "unsupported asset" in canvas_text.lower()
    if not unsupported_asset_warning_contract:
        warnings.append("unsupported-asset warning contract token missing")

    transform_mismatch_section = _contains(cpp_text, ["transform_mismatch", "Transform Mismatch"])
    mesh_reference_mismatch_section = _contains(cpp_text, ["mesh_reference_mismatch", "Mesh Reference Mismatch"])
    if not transform_mismatch_section:
        warnings.append("transform mismatch section is absent")
    if not mesh_reference_mismatch_section:
        warnings.append("mesh-reference mismatch section is absent")

    status = "PASS" if not errors else "FAIL"
    return {
        "status": status,
        "scene_builder_parity_action_present": scene_builder_parity_action_present,
        "parity_status_labels_present": parity_status_labels_present,
        "parity_report_filename_contract": parity_report_filename_contract,
        "parity_report_required_fields": parity_report_required_fields,
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
