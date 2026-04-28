#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "scene_readiness"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


readiness = _load("check_scene_readiness", REPO_ROOT / "scripts" / "check_scene_readiness.py")
paths_report = _load("report_workcell_builder_paths", REPO_ROOT / "scripts" / "report_workcell_builder_paths.py")


class SceneReadinessToolTests(unittest.TestCase):
    def test_check_scene_readiness_passes_on_minimal_valid_fixture(self) -> None:
        payload = readiness.check_readiness(FIXTURES / "valid_workcell", None)
        self.assertEqual(payload["result"], "PASS")

    def test_missing_scenes_directory_fails(self) -> None:
        payload = readiness.check_readiness(FIXTURES / "missing_scenes_workcell", None)
        self.assertEqual(payload["result"], "FAIL")

    def test_absolute_path_warning_detected(self) -> None:
        payload = readiness.check_readiness(FIXTURES / "abs_path_workcell", None)
        self.assertEqual(payload["result"], "WARN")
        self.assertTrue(any("Absolute mesh path" in warning for warning in payload["warnings"]))

    def test_missing_mesh_warning_and_strict_mode_failure(self) -> None:
        warn_payload = readiness.check_readiness(FIXTURES / "missing_mesh_workcell", None)
        strict_payload = readiness.check_readiness(FIXTURES / "missing_mesh_workcell", None, strict=True)
        self.assertEqual(warn_payload["result"], "WARN")
        self.assertEqual(strict_payload["result"], "FAIL")

    def test_duplicate_mesh_basename_warning_detected(self) -> None:
        payload = readiness.check_readiness(FIXTURES / "duplicate_mesh_workcell", None)
        self.assertEqual(payload["result"], "WARN")
        self.assertTrue(any("Duplicate mesh basenames" in warning for warning in payload["warnings"]))

    def test_json_output_is_valid(self) -> None:
        proc = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts" / "check_scene_readiness.py"),
                "--workcell-root",
                str(FIXTURES / "valid_workcell"),
                "--json",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertEqual(payload["result"], "PASS")

    def test_report_workcell_builder_paths_runs(self) -> None:
        payload = paths_report.build_report(REPO_ROOT)
        self.assertIn("workcell_builder_package_path", payload)

    def test_generate_scene_import_checklist_runs(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "checklist.md"
            mesh = FIXTURES / "valid_workcell" / "scenes" / "demo_scene" / "mesh.stl"
            proc = subprocess.run(
                [sys.executable, str(REPO_ROOT / "scripts" / "generate_scene_import_checklist.py"), str(mesh), "--output", str(out)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertTrue(out.is_file())

    def test_environment_layout_to_scene_checklist_runs_with_existing_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "layout_checklist.md"
            fixture = REPO_ROOT / "tests" / "fixtures" / "environment_layouts" / "ur5_table_bins_existing_assets.layout.yaml"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "environment_layout_to_scene_checklist.py"),
                    str(fixture),
                    "--output",
                    str(out),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertTrue(out.is_file())


if __name__ == "__main__":
    unittest.main()
