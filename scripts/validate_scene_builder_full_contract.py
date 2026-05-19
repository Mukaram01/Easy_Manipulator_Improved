#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VALIDATORS = [
    "scripts/validate_scene_builder_mainline_flow.py",
    "scripts/validate_scene_builder_ui_acceptance.py",
    "scripts/validate_scene3d_mesh_preview_contract.py",
    "scripts/validate_scene_builder_canvas_generated_parity.py",
    "scripts/validate_scene_builder_ux_integration.py",
]
SNIPPET_LINES = 6


def _snippet(text: str) -> str:
    lines = [line for line in text.strip().splitlines() if line.strip()]
    if not lines:
        return "(no output)"
    return " | ".join(lines[:SNIPPET_LINES])


def run_validator(path: str) -> dict[str, object]:
    command = [sys.executable, str(ROOT / path)]
    proc = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    merged = "\n".join(part for part in [proc.stdout, proc.stderr] if part)
    return {
        "path": path,
        "exit_code": proc.returncode,
        "status": "PASS" if proc.returncode == 0 else "FAIL",
        "snippet": _snippet(merged),
    }


def print_summary(results: list[dict[str, object]]) -> None:
    print("Scene Builder Full Contract Validation")
    print("=" * 80)
    print(f"{'Validator':62} {'Result':8} Exit  Snippet")
    print("-" * 80)
    for result in results:
        print(
            f"{result['path'][:62]:62} "
            f"{result['status']:8} "
            f"{result['exit_code']!s:4}  "
            f"{result['snippet']}"
        )
    print("-" * 80)
    failures = [r for r in results if r["exit_code"] != 0]
    print(f"Overall: {'PASS' if not failures else 'FAIL'} ({len(results) - len(failures)}/{len(results)} passed)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run full Scene Builder contract validators.")
    parser.add_argument(
        "--validators",
        nargs="*",
        default=DEFAULT_VALIDATORS,
        help="Optional override validator paths (relative to repo root).",
    )
    args = parser.parse_args()

    results = [run_validator(path) for path in args.validators]
    print_summary(results)
    return 0 if all(result["exit_code"] == 0 for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
