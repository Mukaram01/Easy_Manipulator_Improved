#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


audit = _load_module("audit_workcell_large_files", REPO_ROOT / "scripts" / "audit_workcell_large_files.py")


def _cmake_executable_sources(text: str, target: str) -> list[str]:
    match = re.search(rf"add_executable\({re.escape(target)}\s+(.*?)\n\)", text, flags=re.S)
    assert match, f"add_executable({target}) block not found"
    tokens = []
    for raw in match.group(1).splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        tokens.extend(part.strip() for part in stripped.split() if part.strip())
    return tokens


def test_workcell_builder_cmake_has_no_duplicate_target_sources():
    cmake = (REPO_ROOT / "workcell_builder" / "workcell_builder" / "CMakeLists.txt").read_text(encoding="utf-8")
    sources = _cmake_executable_sources(cmake, "workcell_builder")
    duplicates = sorted({source for source in sources if sources.count(source) > 1})
    assert duplicates == []


def test_large_file_audit_reports_known_wrapper_targets_without_failing_ci():
    rows = audit.audit_large_files(REPO_ROOT, warn_lines=1200, critical_lines=2500)
    by_path = {row.path: row for row in rows}

    assert "workcell_builder/workcell_builder/gui/mainwindow.cpp" in by_path
    assert by_path["workcell_builder/workcell_builder/gui/mainwindow.cpp"].status == "critical"
    assert by_path["workcell_builder/workcell_builder/gui/mainwindow.cpp"].wrap_hints

    # This is an audit/report surface for planning the wrapper extraction PRs, not
    # a hard blocker while the product is still being stabilized.
    assert all(row.line_count >= 1200 for row in rows)
