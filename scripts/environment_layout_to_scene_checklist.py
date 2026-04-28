#!/usr/bin/env python3
"""Generate checklist bridging environment_layout/v1 metadata into existing workcell_builder edits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load_layout(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        loaded = json.loads(text)
        if not isinstance(loaded, dict):
            raise ValueError("Layout top-level must be an object")
        return loaded

    try:
        import validate_environment_layout as vel  # local helper already in repo

        loaded, _, _ = vel.load_layout(path)
        if not isinstance(loaded, dict):
            raise ValueError("Layout top-level must be an object")
        return loaded
    except Exception:
        # very small YAML fallback for test-friendly fixtures
        lines = [line.rstrip("\n") for line in text.splitlines() if line.strip() and not line.strip().startswith("#")]
        if lines and lines[0].startswith("{"):
            maybe = json.loads(text)
            if isinstance(maybe, dict):
                return maybe
        raise ValueError("Unable to parse layout; install/use repository yaml tooling.")


def build_checklist(layout: dict[str, Any], source: Path) -> str:
    assets = layout.get("assets") if isinstance(layout.get("assets"), list) else []
    zones = layout.get("zones") if isinstance(layout.get("zones"), list) else []

    lines = [
        "# Environment Layout → Scene Checklist",
        "",
        f"Source layout: `{source}`",
        "",
        "This report is a helper checklist only.",
        "It does **not** generate runtime scenes and does **not** replace workcell_builder.",
        "",
        "## Assets to add/import in existing workcell_builder",
        "",
    ]
    for idx, asset in enumerate(assets):
        if not isinstance(asset, dict):
            continue
        src = asset.get("source") if isinstance(asset.get("source"), dict) else {}
        pose = asset.get("pose") if isinstance(asset.get("pose"), dict) else {}
        lines.extend(
            [
                f"### {idx + 1}. `{asset.get('id', f'asset_{idx}')}`",
                f"- asset_ref: `{asset.get('asset_ref', '-')}`",
                f"- source.path: `{src.get('path', '-')}`",
                f"- source.uri: `{src.get('uri', '-')}`",
                f"- frame: `{pose.get('frame', '-')}`",
                f"- xyz: `{pose.get('xyz', '-')}`",
                f"- rpy: `{pose.get('rpy', '-')}`",
                f"- visual flag: `{asset.get('visual', '-')}`",
                f"- collision flag: `{asset.get('collision', '-')}`",
                "",
            ]
        )

    lines.append("## Zones")
    lines.append("")
    for zone in zones:
        if not isinstance(zone, dict):
            continue
        lines.append(f"- `{zone.get('id', '-')}` type=`{zone.get('type', '-')}` frame=`{zone.get('frame', '-')}` bounds=`{zone.get('bounds_xyz', '-')}`")

    lines.extend(
        [
            "",
            "## Warning",
            "",
            "- This is not runtime scene generation yet.",
            "- Apply these items manually in the existing workcell_builder GUI and save under existing `scenes/`.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout", type=Path)
    parser.add_argument("--output", type=Path, default=Path("reports/environment_layout_scene_checklist.md"))
    args = parser.parse_args()

    layout = _load_layout(args.layout)
    text = build_checklist(layout, args.layout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")
    print(f"PASS: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
