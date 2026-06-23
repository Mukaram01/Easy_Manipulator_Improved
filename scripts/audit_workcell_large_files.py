#!/usr/bin/env python3
"""Report oversized Workcell Studio files that should be split behind wrappers.

This is intentionally an audit/reporting tool, not a hard CI gate.  It gives the
next refactor PR a deterministic target list so the repo can stop growing huge
God-files such as ``gui/mainwindow.cpp`` without doing a dangerous rewrite.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WARN_LINES = 1200
DEFAULT_CRITICAL_LINES = 2500

SCAN_SUFFIXES = {
    ".cpp",
    ".cc",
    ".cxx",
    ".hpp",
    ".h",
    ".hh",
    ".py",
    ".cmake",
    ".txt",
}

SKIP_PARTS = {
    ".git",
    ".pytest_cache",
    "build",
    "install",
    "log",
    "generated",
    "__pycache__",
    "assets",
    "docs",
    "site",
    "dist",
}

WRAP_HINTS = {
    "workcell_builder/workcell_builder/gui/mainwindow.cpp": [
        "scene loading and manifest parsing",
        "Scene3D visual-index ingestion",
        "save/generate action orchestration",
        "task-intent and zone editor wiring",
    ],
    "workcell_builder/workcell_builder/gui/scene3d_viewport_widget.cpp": [
        "mesh loading/cache policy",
        "render diagnostics/counter emission",
        "camera fitting and bounds calculation",
        "selection/picking helpers",
    ],
    "scripts/run_workcell_builder_scene3d_gui_smoke.py": [
        "payload normalization",
        "UR5 final viewport contract checks",
        "wrapper process execution/log capture",
    ],
    "scripts/generate_workcell_from_cell_definition.py": [
        "thin CLI wrapper",
        "package contract renderer",
        "asset/environment renderer",
        "validation/report writer",
    ],
}


@dataclass(frozen=True)
class FileAudit:
    path: str
    line_count: int
    status: str
    wrap_hints: list[str]


def _should_skip(path: Path) -> bool:
    return any(part in SKIP_PARTS for part in path.parts)


def _iter_source_files(root: Path) -> Iterable[Path]:
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if _should_skip(path.relative_to(root)):
            continue
        if path.suffix.lower() in SCAN_SUFFIXES or path.name == "CMakeLists.txt":
            yield path


def _line_count(path: Path) -> int:
    try:
        return len(path.read_text(encoding="utf-8", errors="replace").splitlines())
    except OSError:
        return 0


def audit_large_files(root: Path, warn_lines: int, critical_lines: int) -> list[FileAudit]:
    rows: list[FileAudit] = []
    for path in _iter_source_files(root):
        count = _line_count(path)
        if count < warn_lines:
            continue
        rel = path.relative_to(root).as_posix()
        status = "critical" if count >= critical_lines else "warn"
        rows.append(FileAudit(rel, count, status, WRAP_HINTS.get(rel, [])))
    rows.sort(key=lambda row: (-row.line_count, row.path))
    return rows


def _markdown(rows: list[FileAudit]) -> str:
    lines = [
        "# Workcell Studio Large File Wrap Audit",
        "",
        "This report lists source files that should be wrapped/split before more features are added.",
        "",
        "| Status | Lines | File | First wrapper seams |",
        "| --- | ---: | --- | --- |",
    ]
    for row in rows:
        hints = "; ".join(row.wrap_hints) if row.wrap_hints else "TBD: extract cohesive helper modules before adding more logic"
        lines.append(f"| {row.status} | {row.line_count} | `{row.path}` | {hints} |")
    if not rows:
        lines.append("| pass | 0 | none | no oversized files found |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT)
    parser.add_argument("--warn-lines", type=int, default=DEFAULT_WARN_LINES)
    parser.add_argument("--critical-lines", type=int, default=DEFAULT_CRITICAL_LINES)
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of Markdown")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rows = audit_large_files(args.repo_root.resolve(), args.warn_lines, args.critical_lines)
    if args.json:
        text = json.dumps(
            {
                "schema": "workcell_studio_large_file_audit/v1",
                "warn_lines": args.warn_lines,
                "critical_lines": args.critical_lines,
                "oversized_count": len(rows),
                "files": [asdict(row) for row in rows],
            },
            indent=2,
        ) + "\n"
    else:
        text = _markdown(rows)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
