#!/usr/bin/env python3
"""Smoke-test dry-run sorting plan generator output."""

from pathlib import Path
import subprocess
import sys


def main() -> int:
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "generate_sorting_plan.py"
    )

    result = subprocess.run(
        [sys.executable, str(script_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    output = result.stdout

    expected_mappings = (
        "item_red -> bin_a",
        "item_blue -> bin_b",
        "item_green -> reject_bin",
    )
    for mapping in expected_mappings:
        if mapping not in output:
            raise AssertionError(f"expected mapping not found in output: {mapping}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
