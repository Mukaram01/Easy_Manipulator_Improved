#!/usr/bin/env python3
"""Generate a markdown report for scene manifest contract validation."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import os
import subprocess

REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_CANDIDATES = ("scene_manifest.yaml", "workcell.yaml")
VALIDATOR = REPO_ROOT / "scripts" / "validate_scene_contract.py"
REPORT_PATH = REPO_ROOT / "docs" / "manuals" / "latest_scene_validation_report.md"


@dataclass
class SceneReportRow:
    scene: str
    status: str
    parser: str
    notes: str


def discover_scene_packages() -> list[str]:
    scenes_dir = REPO_ROOT / "scenes"
    if not scenes_dir.is_dir():
        return []

    packages: list[str] = []
    for entry in sorted(scenes_dir.iterdir()):
        if not entry.is_dir():
            continue
        if any((entry / candidate).is_file() for candidate in MANIFEST_CANDIDATES):
            packages.append(entry.name)
    return packages


def summarize_note(text: str, fallback: str) -> str:
    content = text.replace("\n", " ").strip()
    if not content:
        return fallback
    return content if len(content) <= 200 else f"{content[:197]}..."


def resolve_workspace_hint() -> str:
    for key in ("WORKCELL_WS", "WORKSPACE", "COLCON_CURRENT_PREFIX"):
        value = os.environ.get(key)
        if value:
            return value
    return str(Path.cwd())


def _extract_line(prefix: str, output: str) -> str | None:
    for raw in output.splitlines():
        line = raw.strip()
        if line.startswith(prefix):
            return line[len(prefix) :].strip()
    return None


def _collect_messages(output: str) -> list[str]:
    messages: list[str] = []
    for raw in output.splitlines():
        line = raw.strip()
        if line.startswith(("NOTE:", "W", "E")):
            messages.append(line)
    return messages


def validate_scene(package: str) -> SceneReportRow:
    if not VALIDATOR.exists():
        return SceneReportRow(package, "FAIL", "n/a", f"Missing validator at {VALIDATOR}")

    completed = subprocess.run(
        [str(VALIDATOR), package],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    output = (completed.stdout or "") + (completed.stderr or "")

    result = _extract_line("RESULT:", output) or "FAIL"
    parser_name = _extract_line("Parser:", output) or "unknown"
    notes = summarize_note(" | ".join(_collect_messages(output)), "Validation completed.")

    if completed.returncode == 3:
        return SceneReportRow(package, "SKIP", parser_name, notes)

    if completed.returncode != 0:
        return SceneReportRow(package, "FAIL", parser_name, notes)

    if result == "WARN":
        return SceneReportRow(package, "WARN", parser_name, notes)

    return SceneReportRow(package, "PASS", parser_name, notes)


def main() -> int:
    packages = discover_scene_packages()
    if not packages:
        print("No scene packages found under ./scenes; report not generated.")
        return 2

    rows = [validate_scene(package) for package in packages]

    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now(timezone.utc).isoformat()
    workspace_hint = resolve_workspace_hint()

    parser_summary = sorted({row.parser for row in rows})

    lines = [
        "# Latest Scene Validation Report",
        "",
        f"- Generated (UTC): `{timestamp}`",
        f"- Repository: `{REPO_ROOT}`",
        f"- Workspace hint: `{workspace_hint}`",
        f"- Parser backend(s): `{', '.join(parser_summary)}`",
        "",
        "## Scene contract results",
        "",
        "| scene | status | parser | notes |",
        "|---|---|---|---|",
    ]

    for row in rows:
        lines.append(f"| `{row.scene}` | **{row.status}** | `{row.parser}` | {row.notes} |")

    pass_count = sum(1 for row in rows if row.status == "PASS")
    fail_count = sum(1 for row in rows if row.status == "FAIL")
    skip_count = sum(1 for row in rows if row.status == "SKIP")
    warn_count = sum(1 for row in rows if row.status == "WARN")

    next_action = "- Safe to proceed to launch smoke tests."
    if fail_count > 0:
        failed = ", ".join(row.scene for row in rows if row.status == "FAIL")
        next_action = (
            "- Run `./scripts/validate_scene_contract.py <scene_name>` for failed scenes: "
            f"`{failed}`."
        )
    elif skip_count > 0 and fail_count == 0 and pass_count == 0 and warn_count == 0:
        next_action = "- Source workspace and rebuild/install scenes, then rerun preflight."

    lines.extend(
        [
            "",
            "## Summary",
            "",
            f"- PASS: {pass_count}",
            f"- FAIL: {fail_count}",
            f"- SKIP: {skip_count}",
            f"- WARN: {warn_count}",
            "",
            "## Next action",
            "",
            next_action,
            "",
            "Generated by `./scripts/generate_scene_validation_report.py`.",
        ]
    )

    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote scene validation report: {REPORT_PATH}")

    return 1 if fail_count > 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
