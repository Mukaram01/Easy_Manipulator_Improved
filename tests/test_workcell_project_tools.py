#!/usr/bin/env python3
"""Tests for workcell project generator/check helper tooling."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures"
GENERATOR = REPO_ROOT / "scripts" / "create_workcell_project.py"
CHECKER = REPO_ROOT / "scripts" / "check_workcell_projects.sh"
DASHBOARD_GENERATOR = REPO_ROOT / "scripts" / "generate_workcell_dashboard.py"


class WorkcellProjectGeneratorTests(unittest.TestCase):
    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(GENERATOR), *args], capture_output=True, text=True, check=False)

    def _run_dashboard(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(DASHBOARD_GENERATOR), *args], capture_output=True, text=True, check=False)

    def test_dry_run_writes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_colour.yaml"),
                "--output-dir", str(out),
                "--dry-run",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertFalse((out / "ur5_colour_sort_cell").exists())
            self.assertIn("dry-run", proc.stdout)

    def test_project_from_fixture_succeeds_with_required_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_colour.yaml"),
                "--output-dir", str(out),
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            project_dir = out / "ur5_colour_sort_cell"
            manifest = project_dir / "project_manifest.json"
            self.assertTrue(manifest.is_file())
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            pkg = payload["generated_package_name"]
            self.assertTrue((project_dir / "README.md").is_file())
            self.assertTrue((project_dir / "cell_definition.yaml").is_file())
            self.assertTrue((project_dir / "next_commands.md").is_file())
            self.assertTrue((project_dir / "generated_workcell" / pkg / "scene_manifest.yaml").is_file())
            self.assertTrue((project_dir / "reports" / "validation_summary.md").is_file())
            self.assertTrue((project_dir / "reports" / "task_recipe_dry_run.md").is_file())

    def test_project_manifest_contains_required_fields(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_pick_place.yaml"),
                "--output-dir", str(out),
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            payload = json.loads((out / "ur5_2f_demo_cell" / "project_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], "workcell_project/v1")
            self.assertIn("statuses", payload)
            self.assertIn("artifacts", payload)
            self.assertIn("checksums", payload)
            self.assertIn("cell_definition_validation", payload["statuses"])

    def test_no_overwrite_without_force(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            args = [
                "--cell-definition", str(FIXTURES / "cell_definition_pick_place.yaml"),
                "--output-dir", str(out),
            ]
            first = self._run(*args, "--force")
            self.assertEqual(first.returncode, 0, msg=first.stdout + first.stderr)
            second = self._run(*args)
            self.assertNotEqual(second.returncode, 0)
            self.assertIn("already exists", second.stdout)

    def test_overwrite_with_force(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            args = [
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_shape.yaml"),
                "--output-dir", str(out),
            ]
            first = self._run(*args, "--force")
            self.assertEqual(first.returncode, 0, msg=first.stdout + first.stderr)
            readme = out / "ur5_shape_sort_cell" / "README.md"
            readme.write_text("old", encoding="utf-8")
            second = self._run(*args, "--force")
            self.assertEqual(second.returncode, 0, msg=second.stdout + second.stderr)
            self.assertNotEqual(readme.read_text(encoding="utf-8"), "old")

    def test_invalid_cell_definition_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            invalid = out / "invalid.yaml"
            invalid.write_text("schema_version: cell_definition/v1\ncell: {}\n", encoding="utf-8")
            proc = self._run("--cell-definition", str(invalid), "--output-dir", str(out))
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("invalid cell definition", proc.stdout)

    def test_missing_input_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            proc = self._run("--cell-definition", str(Path(tmpdir) / "missing.yaml"), "--output-dir", tmpdir)
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("missing cell definition", proc.stdout)

    def test_skip_bundle_and_skip_execution_plan(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_garbage_sorting.yaml"),
                "--output-dir", str(out),
                "--skip-bundle",
                "--skip-execution-plan",
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            manifest = json.loads((out / "ur5_garbage_sort_cell" / "project_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["statuses"]["commissioning_bundle"], "SKIP")
            self.assertEqual(manifest["statuses"]["task_execution_plan"], "SKIP")

    def test_strict_converts_warn_or_skip_to_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_colour.yaml"),
                "--output-dir", str(out),
                "--skip-bundle",
                "--strict",
                "--force",
            )
            self.assertNotEqual(proc.returncode, 0)

    def test_generated_readme_and_next_commands_include_launch_and_validation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_pick_place.yaml"),
                "--output-dir", str(out),
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            project = out / "ur5_2f_demo_cell"
            readme = (project / "README.md").read_text(encoding="utf-8")
            next_cmds = (project / "next_commands.md").read_text(encoding="utf-8")
            self.assertIn("ros2 launch run_grasp_execution grasp_execution.launch.py scene_package:=", readme)
            self.assertIn("validate_scene_contract.py", readme)
            self.assertIn("python3 scripts/validate_cell_definition.py", next_cmds)
            self.assertIn("colcon build --packages-select", next_cmds)

    def test_checksums_generated_and_direct_validation_noted(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_shape.yaml"),
                "--output-dir", str(out),
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            project = out / "ur5_shape_sort_cell"
            manifest = json.loads((project / "project_manifest.json").read_text(encoding="utf-8"))
            self.assertTrue(manifest["checksums"])
            validation_summary = (project / "reports" / "validation_summary.md").read_text(encoding="utf-8")
            self.assertIn("direct file", validation_summary)

    def test_dashboard_generated_by_default_and_manifest_includes_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run("--cell-definition", str(FIXTURES / "cell_definition_pick_place.yaml"), "--output-dir", str(out), "--force")
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            project = out / "ur5_2f_demo_cell"
            dashboard = project / "dashboard" / "index.html"
            self.assertTrue(dashboard.is_file())
            html_text = dashboard.read_text(encoding="utf-8")
            self.assertIn("Workcell Project Dashboard", html_text)
            manifest = json.loads((project / "project_manifest.json").read_text(encoding="utf-8"))
            self.assertIn("dashboard", manifest["artifacts"])
            self.assertIn("dashboard", manifest["checksums"])

    def test_skip_dashboard_prevents_creation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            proc = self._run(
                "--cell-definition", str(FIXTURES / "cell_definition_sort_by_colour.yaml"),
                "--output-dir", str(out),
                "--skip-dashboard",
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            project = out / "ur5_colour_sort_cell"
            self.assertFalse((project / "dashboard" / "index.html").exists())

    def test_generate_dashboard_from_project_and_manifest_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            create = self._run("--cell-definition", str(FIXTURES / "cell_definition_sort_by_shape.yaml"), "--output-dir", str(out), "--force")
            self.assertEqual(create.returncode, 0, msg=create.stdout + create.stderr)
            project = out / "ur5_shape_sort_cell"
            manifest = project / "project_manifest.json"
            dash_a = project / "dashboard" / "a.html"
            dash_b = project / "dashboard" / "b.html"
            proc_a = self._run_dashboard("--project-dir", str(project), "--output", str(dash_a))
            self.assertEqual(proc_a.returncode, 0, msg=proc_a.stdout + proc_a.stderr)
            proc_b = self._run_dashboard("--manifest", str(manifest), "--output", str(dash_b))
            self.assertEqual(proc_b.returncode, 0, msg=proc_b.stdout + proc_b.stderr)
            self.assertIn("Project Summary", dash_a.read_text(encoding="utf-8"))
            self.assertIn("Generated Artifacts", dash_b.read_text(encoding="utf-8"))

    def test_dashboard_escapes_unsafe_html(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            project = Path(tmpdir) / "proj"
            project.mkdir(parents=True)
            manifest = project / "project_manifest.json"
            payload = {
                "schema_version": "workcell_project/v1",
                "cell_name": "<script>alert('x')</script>",
                "generated_package_name": "pkg",
                "statuses": {"scene_manifest_validation": "PASS"},
                "artifacts": {"next_commands": "next_commands.md"},
            }
            manifest.write_text(json.dumps(payload), encoding="utf-8")
            (project / "next_commands.md").write_text("<b>danger</b>", encoding="utf-8")
            dash = project / "dashboard" / "index.html"
            proc = self._run_dashboard("--manifest", str(manifest), "--output", str(dash))
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            text = dash.read_text(encoding="utf-8")
            self.assertIn("&lt;script&gt;alert", text)
            self.assertNotIn("<script>alert", text)
            self.assertIn("&lt;b&gt;danger&lt;/b&gt;", text)

    def test_dashboard_missing_optional_fields_warn_and_strict_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            project = Path(tmpdir) / "p"
            project.mkdir(parents=True)
            manifest = project / "project_manifest.json"
            manifest.write_text(json.dumps({"cell_name": "x"}), encoding="utf-8")
            proc_warn = self._run_dashboard("--manifest", str(manifest))
            self.assertEqual(proc_warn.returncode, 0, msg=proc_warn.stdout + proc_warn.stderr)
            self.assertIn("WARN", proc_warn.stdout)
            proc_strict = self._run_dashboard("--manifest", str(manifest), "--strict")
            self.assertNotEqual(proc_strict.returncode, 0)

    def test_dashboard_json_summary(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir)
            create = self._run("--cell-definition", str(FIXTURES / "cell_definition_pick_place.yaml"), "--output-dir", str(out), "--force")
            self.assertEqual(create.returncode, 0, msg=create.stdout + create.stderr)
            project = out / "ur5_2f_demo_cell"
            proc = self._run_dashboard("--project-dir", str(project), "--json", "--quiet")
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            payload = json.loads(proc.stdout)
            self.assertIn("dashboard_path", payload)
            self.assertIn("dashboard_checksum", payload)


class WorkcellProjectCheckHelperTests(unittest.TestCase):
    def test_check_helper_returns_parseable_pass_warn_fail_summary(self) -> None:
        proc = subprocess.run(["bash", str(CHECKER)], capture_output=True, text=True, check=False)
        self.assertIn("Workcell project checks:", proc.stdout)
        self.assertRegex(proc.stdout, r"Workcell project checks: (PASS|WARN|FAIL)")
        self.assertIn(proc.returncode, (0, 1))


if __name__ == "__main__":
    unittest.main()
