#!/usr/bin/env python3
"""Classify colcon build warnings into owned/third-party/unknown buckets."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Dict, List

KNOWN_THIRD_PARTY_LABEL = "known_third_party"
OWNED_LABEL = "owned_warnings"
UNKNOWN_LABEL = "unknown_warnings"

_WARNING_LINE_RE = re.compile(r"(?i)(warning:|\bwarning\b|\bnote\b)")
_OWNED_HINTS = (
    "easy_manipulation_deployment/",
    "run_grasp_execution",
    "run_grasp_planner",
    "run_waypoint_execution",
    "workcell_builder",
    "scenario",
)


def _is_osqp_dev_warning(window: str) -> bool:
    return "OSQP_EIGEN_OSQP_TARGET_TO_LINK" in window and "OsqpEigenDependencies.cmake" in window


def _is_tesseract_assimp_warning(window: str) -> bool:
    return (
        "pbrmaterial.h is deprecated" in window
        and "/usr/include/assimp/pbrmaterial.h" in window
        and any(pkg in window for pkg in ("tesseract_geometry", "tesseract_urdf", "tesseract_examples"))
    )


def _is_owned_warning(window: str) -> bool:
    lower = window.lower()
    return any(hint in lower for hint in _OWNED_HINTS)


def classify_warnings(log_text: str) -> Dict[str, List[str]]:
    lines = log_text.splitlines()
    classified: Dict[str, List[str]] = {
        KNOWN_THIRD_PARTY_LABEL: [],
        OWNED_LABEL: [],
        UNKNOWN_LABEL: [],
    }

    for index, line in enumerate(lines):
        if not _WARNING_LINE_RE.search(line):
            continue

        context = "\n".join(lines[max(0, index - 2) : min(len(lines), index + 3)])
        snippet = context.strip()
        if _is_osqp_dev_warning(context):
            classified[KNOWN_THIRD_PARTY_LABEL].append(snippet)
        elif _is_tesseract_assimp_warning(context):
            classified[KNOWN_THIRD_PARTY_LABEL].append(snippet)
        elif _is_owned_warning(context):
            classified[OWNED_LABEL].append(snippet)
        else:
            classified[UNKNOWN_LABEL].append(snippet)

    return classified


def _build_result(classified: Dict[str, List[str]]) -> Dict[str, object]:
    return {
        "counts": {key: len(value) for key, value in classified.items()},
        "matches": classified,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, help="Path to colcon build log.")
    parser.add_argument("--fail-on-owned", action="store_true", help="Fail on owned or unknown warnings.")
    parser.add_argument("--json", action="store_true", help="Emit JSON report.")
    args = parser.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        print(f"Log file not found: {log_path}", file=sys.stderr)
        return 2

    classified = classify_warnings(log_path.read_text(encoding="utf-8", errors="replace"))
    result = _build_result(classified)

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print("Warning classification summary:")
        for key, count in result["counts"].items():
            print(f"  {key}: {count}")

    if args.fail_on_owned:
        if classified[OWNED_LABEL] or classified[UNKNOWN_LABEL]:
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
