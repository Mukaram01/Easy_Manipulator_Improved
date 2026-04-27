#!/usr/bin/env python3
"""Generate deterministic scene self-test metadata reports."""

from __future__ import annotations

import argparse
import importlib.util
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_CANDIDATES = ("scene_manifest.yaml", "workcell.yaml")
REPORT_PATH = REPO_ROOT / "docs" / "manuals" / "latest_scene_self_test_report.md"
VALIDATOR_PATH = REPO_ROOT / "scripts" / "validate_scene_contract.py"


def _load_validator_module():
    spec = importlib.util.spec_from_file_location("validate_scene_contract", VALIDATOR_PATH)
    if not spec or not spec.loader:
        raise RuntimeError(f"Unable to load validator module from {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["validate_scene_contract"] = module
    spec.loader.exec_module(module)
    return module


validator = _load_validator_module()


@dataclass
class SelfTestRow:
    scene: str
    status: str
    object_id: str
    frame: str
    pose_xyz: str
    dimensions: str
    notes: str


def discover_scene_manifests() -> list[tuple[str, Path]]:
    scenes_dir = REPO_ROOT / "scenes"
    if not scenes_dir.is_dir():
        return []

    discovered: list[tuple[str, Path]] = []
    for entry in sorted(scenes_dir.iterdir()):
        if not entry.is_dir():
            continue
        for candidate in MANIFEST_CANDIDATES:
            path = entry / candidate
            if path.is_file():
                discovered.append((entry.name, path))
                break
    return discovered


def _is_numeric_list(value: Any, expected_len: int, positive_only: bool = False) -> bool:
    if not isinstance(value, list) or len(value) != expected_len:
        return False
    for item in value:
        if not isinstance(item, (int, float)):
            return False
        if isinstance(item, float) and not math.isfinite(item):
            return False
        if positive_only and item <= 0:
            return False
    return True


def _format_list(value: Any) -> str:
    if not isinstance(value, list):
        return "(n/a)"
    return "[" + ", ".join(str(v) for v in value) + "]"


def validate_self_test_block(manifest: dict[str, Any]) -> tuple[str, list[str]]:
    notes: list[str] = []

    self_test = manifest.get("self_test")
    if self_test is None:
        return "WARN", ["self_test not defined; add commissioning object metadata."]

    if not isinstance(self_test, dict):
        return "FAIL", ["self_test must be a mapping/object when provided."]

    enabled = self_test.get("enabled")
    if enabled is None:
        return "WARN", ["self_test.enabled missing; treated as disabled."]
    if not isinstance(enabled, bool):
        return "FAIL", ["self_test.enabled must be boolean."]
    if not enabled:
        return "WARN", ["self_test.enabled=false; metadata intentionally disabled."]

    obj = self_test.get("object")
    if not isinstance(obj, dict):
        notes.append("self_test.object must be a mapping/object.")
    else:
        if not isinstance(obj.get("id"), str) or not obj.get("id", "").strip():
            notes.append("self_test.object.id must be non-empty string.")
        if obj.get("shape") != "box":
            notes.append("self_test.object.shape currently supports only 'box'.")
        if not _is_numeric_list(obj.get("dimensions"), 3, positive_only=True):
            notes.append("self_test.object.dimensions must be [x,y,z] numeric > 0.")
        if not isinstance(obj.get("frame_id"), str) or not obj.get("frame_id", "").strip():
            notes.append("self_test.object.frame_id must be non-empty string.")
        if not _is_numeric_list(obj.get("pose_xyz"), 3):
            notes.append("self_test.object.pose_xyz must be numeric list length 3.")
        if not _is_numeric_list(obj.get("pose_rpy"), 3):
            notes.append("self_test.object.pose_rpy must be numeric list length 3.")

    expected = self_test.get("expected")
    if expected is not None:
        if not isinstance(expected, dict):
            notes.append("self_test.expected must be a mapping/object when provided.")
        else:
            min_candidates = expected.get("min_grasp_candidates")
            if min_candidates is not None and (
                not isinstance(min_candidates, int) or min_candidates < 1
            ):
                notes.append("self_test.expected.min_grasp_candidates must be integer >= 1.")
            allow_sim = expected.get("allow_simulated_execution")
            if allow_sim is not None and not isinstance(allow_sim, bool):
                notes.append("self_test.expected.allow_simulated_execution must be boolean.")

    if notes:
        return "FAIL", notes
    return "PASS", ["Self-test metadata is present and valid."]


def evaluate_scene(scene_name: str, manifest_path: Path) -> SelfTestRow:
    try:
        manifest, _, parser_notes = validator._read_manifest(str(manifest_path))
    except Exception as exc:  # pragma: no cover - exercised by integration workflows
        return SelfTestRow(
            scene=scene_name,
            status="FAIL",
            object_id="(n/a)",
            frame="(n/a)",
            pose_xyz="(n/a)",
            dimensions="(n/a)",
            notes=f"Manifest parse failed: {exc}",
        )

    status, validation_notes = validate_self_test_block(manifest)
    self_test_obj = ((manifest.get("self_test") or {}).get("object") or {})

    notes = parser_notes + validation_notes
    return SelfTestRow(
        scene=scene_name,
        status=status,
        object_id=str(self_test_obj.get("id", "(n/a)")),
        frame=str(self_test_obj.get("frame_id", "(n/a)")),
        pose_xyz=_format_list(self_test_obj.get("pose_xyz")),
        dimensions=_format_list(self_test_obj.get("dimensions")),
        notes="; ".join(notes),
    )


def build_report(rows: list[SelfTestRow]) -> str:
    timestamp = datetime.now(timezone.utc).isoformat()
    lines = [
        "# Latest Scene Self-Test Metadata Report",
        "",
        f"- Generated (UTC): `{timestamp}`",
        f"- Repository: `{REPO_ROOT}`",
        "",
        "## Scene self-test status",
        "",
        "| scene | status | object id | frame | pose_xyz | dimensions | notes / next action |",
        "|---|---|---|---|---|---|---|",
    ]

    for row in rows:
        lines.append(
            f"| `{row.scene}` | **{row.status}** | `{row.object_id}` | `{row.frame}` | "
            f"`{row.pose_xyz}` | `{row.dimensions}` | {row.notes} |"
        )

    pass_count = sum(1 for row in rows if row.status == "PASS")
    warn_count = sum(1 for row in rows if row.status == "WARN")
    fail_count = sum(1 for row in rows if row.status == "FAIL")

    lines.extend(
        [
            "",
            "## Summary",
            "",
            f"- PASS: {pass_count}",
            f"- WARN: {warn_count}",
            f"- FAIL: {fail_count}",
            "",
            "Generated by `./scripts/generate_scene_self_test_report.py`.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenes", nargs="*", help="Optional scene names to include.")
    parser.add_argument("--check", action="store_true", help="Print PASS/WARN/FAIL summary to stdout.")
    args = parser.parse_args()

    discovered = discover_scene_manifests()
    if args.scenes:
        requested = set(args.scenes)
        discovered = [entry for entry in discovered if entry[0] in requested]

    if not discovered:
        print("No scene manifests discovered for self-test reporting.")
        return 2

    rows = [evaluate_scene(scene, path) for scene, path in discovered]

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(build_report(rows), encoding="utf-8")
    print(f"Wrote scene self-test report: {REPORT_PATH}")

    if args.check:
        for row in rows:
            print(f"{row.status:4} {row.scene:24} {row.notes}")
        pass_count = sum(1 for row in rows if row.status == "PASS")
        warn_count = sum(1 for row in rows if row.status == "WARN")
        fail_count = sum(1 for row in rows if row.status == "FAIL")
        print(f"Summary: PASS={pass_count} WARN={warn_count} FAIL={fail_count}")

    return 1 if any(row.status == "FAIL" for row in rows) else 0


if __name__ == "__main__":
    raise SystemExit(main())
