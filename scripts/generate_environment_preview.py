#!/usr/bin/env python3
"""Generate markdown preview for environment_layout/v1."""

from __future__ import annotations

import argparse
from pathlib import Path

import validate_environment_layout as layout_validator

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "reports" / "environment_layout_preview.md"


def render_preview(layout: dict, validation: layout_validator.ValidationResult, source_path: Path) -> str:
    assets = layout.get("assets") if isinstance(layout.get("assets"), list) else []
    zones = layout.get("zones") if isinstance(layout.get("zones"), list) else []
    safety = layout.get("safety") if isinstance(layout.get("safety"), dict) else {}
    safety_zones = safety.get("zones") if isinstance(safety.get("zones"), list) else []
    lines = [
        "# Environment Layout Preview",
        "",
        f"- Source: `{source_path}`",
        f"- Layout id: `{layout.get('layout_id', '(unknown)')}`",
        f"- Schema version: `{layout.get('schema_version', '(missing)')}`",
        f"- Validation parser: `{validation.parser}`",
        "",
        "## Assets",
        "",
        "| id | type | asset_ref | source | pose xyz | pose rpy |",
        "|---|---|---|---|---|---|",
    ]
    for asset in assets:
        if not isinstance(asset, dict):
            continue
        src = asset.get("source") if isinstance(asset.get("source"), dict) else {}
        source = src.get("uri") or src.get("path") or src.get("package") or "-"
        pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else {}
        lines.append(
            f"| {asset.get('id', '-')} | {asset.get('type', '-')} | {asset.get('asset_ref', '-')} | "
            f"`{source}` | `{pose.get('xyz', '-')}` | `{pose.get('rpy', '-')}` |"
        )
    if len(lines) == 11:
        lines.append("| - | - | - | - | - | - |")

    lines.extend(["", "## Zones", ""])
    for zone in zones:
        if not isinstance(zone, dict):
            continue
        lines.append(f"- `{zone.get('id', '-')}` ({zone.get('type', '-')}) bounds={zone.get('bounds_xyz', {})}")

    lines.extend(["", "## Safety zones", ""])
    for zone in safety_zones:
        if not isinstance(zone, dict):
            continue
        lines.append(f"- `{zone.get('id', '-')}` ({zone.get('type', '-')}) bounds={zone.get('bounds_xyz', {})}")

    lines.extend(["", "## Validation warnings", ""])
    if validation.warnings:
        lines.extend([f"- {warning}" for warning in validation.warnings])
    else:
        lines.append("- None")

    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout_path", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    loaded, parser_name, notes = layout_validator.load_layout(args.layout_path)
    summary = layout_validator.validate_layout(loaded, args.layout_path, parser_name, notes, strict=False)
    text = render_preview(loaded, summary, args.layout_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"PASS: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
