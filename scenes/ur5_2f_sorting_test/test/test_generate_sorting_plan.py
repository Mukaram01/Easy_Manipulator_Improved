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

    if "item_green -> reject_bin" not in output:
        raise AssertionError("item_green reject route not present in dry-run output")

    if "destination_frame: bin_a" in output.split("item_green -> reject_bin", maxsplit=1)[-1]:
        raise AssertionError("item_green resolved to wrong destination frame (bin_a)")

    if "destination_frame: reject_bin" not in output:
        raise AssertionError("item_green did not resolve to destination_frame: reject_bin")

    return 0


if __name__ == "__main__":
    sys.exit(main())
