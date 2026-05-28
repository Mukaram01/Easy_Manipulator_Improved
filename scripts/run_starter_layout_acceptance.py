#!/usr/bin/env python3
"""Run the offline Workcell Studio starter-layout acceptance test.

The acceptance coverage lives in the workcell_builder C++ gtest so it can call
build_workcell_studio_canvas_model() and build_starter_layout_entries_from_preview()
directly. This wrapper only locates and invokes that already-built test binary.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


TEST_NAME = "*StarterLayoutAcceptanceCopiesSceneAndFiltersUnsafePreviewItems"
BINARY_NAME = "workcell_studio_canvas_model_mesh_test"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def candidate_binaries(root: Path) -> list[Path]:
    patterns = [
        f"build/**/{BINARY_NAME}",
        f"install/**/{BINARY_NAME}",
        f"**/{BINARY_NAME}",
    ]
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(path for path in root.glob(pattern) if path.is_file() and path.stat().st_mode & 0o111)
    return sorted(set(candidates), key=lambda p: (len(p.parts), str(p)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Run starter-layout generation acceptance coverage without hardware.")
    parser.add_argument("--test-binary", type=Path, help="Path to an already-built workcell_studio_canvas_model_mesh_test binary.")
    args = parser.parse_args()

    root = repo_root()
    test_binary = args.test_binary
    if test_binary is None:
        candidates = candidate_binaries(root)
        if not candidates:
            print(
                "Could not find an executable workcell_studio_canvas_model_mesh_test binary. "
                "Build the workcell_builder tests first, or pass --test-binary.",
                file=sys.stderr,
            )
            return 2
        test_binary = candidates[0]

    command = [str(test_binary), f"--gtest_filter={TEST_NAME}"]
    print("Running:", " ".join(command))
    return subprocess.call(command, cwd=root)


if __name__ == "__main__":
    raise SystemExit(main())
