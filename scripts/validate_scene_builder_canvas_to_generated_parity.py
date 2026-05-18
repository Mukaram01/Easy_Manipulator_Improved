#!/usr/bin/env python3
"""Deprecated compatibility wrapper for the canonical Scene Builder parity validator."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
CANONICAL_SCRIPT = SCRIPT_DIR / "validate_scene_builder_canvas_generated_parity.py"


def main() -> int:
    print(
        "DEPRECATED: scripts/validate_scene_builder_canvas_to_generated_parity.py has been retired. "
        "Use scripts/validate_scene_builder_canvas_generated_parity.py instead.",
        file=sys.stderr,
    )

    if not CANONICAL_SCRIPT.is_file():
        print(f"ERROR: canonical validator not found at {CANONICAL_SCRIPT}", file=sys.stderr)
        return 2

    cmd = [sys.executable, str(CANONICAL_SCRIPT), *sys.argv[1:]]
    completed = subprocess.run(cmd, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
