#!/usr/bin/env python3
"""Dependency-light tests for scene validation/report scripts."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


validator = _load_module("validate_scene_contract", REPO_ROOT / "scripts" / "validate_scene_contract.py")
reporter = _load_module(
    "generate_scene_validation_report", REPO_ROOT / "scripts" / "generate_scene_validation_report.py"
)
smoke_reporter = _load_module(
    "generate_smoke_launch_report", REPO_ROOT / "scripts" / "generate_smoke_launch_report.py"
)
self_test_reporter = _load_module(
    "generate_scene_self_test_report", REPO_ROOT / "scripts" / "generate_scene_self_test_report.py"
)


class FallbackYamlParserTests(unittest.TestCase):
    def test_representative_manifest_parses(self) -> None:
        manifest_text = """
scene:
  name: ur5_2f_test
robot:
  model: ur5
  planning_group: manipulator
  base_frame: base_link
  ee_link: tool0
end_effector:
  grasp_frame: tool0
  allowed_touch_links:
    - tool0
perception:
  input_frame_options: [world, base_link]
home_return:
  safe_joint_state: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
"""
        parsed = validator.parse_manifest_yaml(manifest_text)
        self.assertEqual(parsed["scene"]["name"], "ur5_2f_test")
        self.assertEqual(parsed["perception"]["input_frame_options"], ["world", "base_link"])
        self.assertEqual(len(parsed["home_return"]["safe_joint_state"]), 6)

    def test_block_list_parses(self) -> None:
        parsed = validator.parse_manifest_yaml(
            """
end_effector:
  allowed_touch_links:
    - link_a
    - link_b
"""
        )
        self.assertEqual(parsed["end_effector"]["allowed_touch_links"], ["link_a", "link_b"])

    def test_unsupported_yaml_syntax_raises_clear_error(self) -> None:
        with self.assertRaises(validator.SimpleYamlError):
            validator.parse_manifest_yaml(
                """
end_effector:
  allowed_touch_links:
    - name: link_a
"""
            )

    def test_read_manifest_uses_fallback_when_pyyaml_missing(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write("robot:\n  ee_link: tool0\n")
            path = handle.name

        with mock.patch.object(validator, "_pyyaml", None):
            loaded, parser, notes = validator._read_manifest(path)

        self.assertEqual(parser, "fallback")
        self.assertIn("PyYAML not available", " ".join(notes))
        self.assertEqual(loaded["robot"]["ee_link"], "tool0")


class ReportGenerationTests(unittest.TestCase):
    def test_validate_scene_parses_fallback_output_without_pyyaml(self) -> None:
        fake_output = "\n".join(
            [
                "Scene package: ur5_2f_test",
                "Parser: fallback",
                "NOTE: PyYAML not available: using built-in fallback parser.",
                "RESULT: PASS",
            ]
        )
        completed = mock.Mock(returncode=0, stdout=fake_output, stderr="")

        with mock.patch.object(reporter.subprocess, "run", return_value=completed):
            row = reporter.validate_scene("ur5_2f_test")

        self.assertEqual(row.status, "PASS")
        self.assertEqual(row.parser, "fallback")

    def test_discover_scene_packages_from_manifest_candidates(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_root = Path(tmpdir)
            (tmp_root / "scenes" / "scene_a").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_b").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_c").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_a" / "scene_manifest.yaml").write_text(
                "scene: {}\n", encoding="utf-8"
            )
            (tmp_root / "scenes" / "scene_b" / "workcell.yaml").write_text(
                "scene: {}\n", encoding="utf-8"
            )

            with mock.patch.object(reporter, "REPO_ROOT", tmp_root):
                discovered = reporter.discover_scene_packages()

        self.assertEqual(discovered, ["scene_a", "scene_b"])


class SelfTestValidationTests(unittest.TestCase):
    def test_valid_self_test_block(self) -> None:
        manifest = {
            "self_test": {
                "enabled": True,
                "object": {
                    "id": "commissioning_box",
                    "shape": "box",
                    "dimensions": [0.05, 0.05, 0.05],
                    "frame_id": "world",
                    "pose_xyz": [0.45, 0.0, 0.08],
                    "pose_rpy": [0.0, 0.0, 0.0],
                },
                "expected": {"min_grasp_candidates": 1, "allow_simulated_execution": True},
            }
        }
        status, notes = self_test_reporter.validate_self_test_block(manifest)
        self.assertEqual(status, "PASS")
        self.assertIn("present and valid", " ".join(notes))

    def test_missing_self_test_warns(self) -> None:
        status, notes = self_test_reporter.validate_self_test_block({})
        self.assertEqual(status, "WARN")
        self.assertIn("not defined", " ".join(notes))

    def test_invalid_dimensions_fail(self) -> None:
        manifest = {
            "self_test": {
                "enabled": True,
                "object": {
                    "id": "commissioning_box",
                    "shape": "box",
                    "dimensions": [0.05, -0.01, 0.05],
                    "frame_id": "world",
                    "pose_xyz": [0.45, 0.0, 0.08],
                    "pose_rpy": [0.0, 0.0, 0.0],
                },
            }
        }
        status, notes = self_test_reporter.validate_self_test_block(manifest)
        self.assertEqual(status, "FAIL")
        self.assertIn("dimensions", " ".join(notes))

    def test_invalid_pose_length_fail(self) -> None:
        manifest = {
            "self_test": {
                "enabled": True,
                "object": {
                    "id": "commissioning_box",
                    "shape": "box",
                    "dimensions": [0.05, 0.05, 0.05],
                    "frame_id": "world",
                    "pose_xyz": [0.45, 0.0],
                    "pose_rpy": [0.0, 0.0, 0.0],
                },
            }
        }
        status, notes = self_test_reporter.validate_self_test_block(manifest)
        self.assertEqual(status, "FAIL")
        self.assertIn("pose_xyz", " ".join(notes))

    def test_report_generation_with_fake_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_root = Path(tmpdir)
            (tmp_root / "scenes" / "scene_a").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_b").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_a" / "scene_manifest.yaml").write_text(
                "self_test:\n"
                "  enabled: true\n"
                "  object:\n"
                "    id: commissioning_box\n"
                "    shape: box\n"
                "    dimensions: [0.05, 0.05, 0.05]\n"
                "    frame_id: world\n"
                "    pose_xyz: [0.45, 0.0, 0.08]\n"
                "    pose_rpy: [0.0, 0.0, 0.0]\n",
                encoding="utf-8",
            )
            (tmp_root / "scenes" / "scene_b" / "workcell.yaml").write_text(
                "scene:\n  name: scene_b\n",
                encoding="utf-8",
            )

            with mock.patch.object(self_test_reporter, "REPO_ROOT", tmp_root):
                with mock.patch.object(
                    self_test_reporter, "REPORT_PATH", tmp_root / "docs" / "manuals" / "report.md"
                ):
                    discovered = self_test_reporter.discover_scene_manifests()
                    rows = [self_test_reporter.evaluate_scene(name, path) for name, path in discovered]
                    report_text = self_test_reporter.build_report(rows)

        self.assertEqual(len(rows), 2)
        self.assertIn("`scene_a` | **PASS**", report_text)
        self.assertIn("`scene_b` | **WARN**", report_text)


class SmokeReportTests(unittest.TestCase):
    def test_parse_smoke_results(self) -> None:
        rows = smoke_reporter.parse_smoke_results(
            "\n".join(
                [
                    "ur5_2f_test\tPASS\tGenerated workcell context for scene\tReady\t/tmp/ur5_2f_test.log",
                    "suction_test\tSKIP\t\tNot installed\t/tmp/suction_test.log",
                ]
            )
        )
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0].status, "PASS")
        self.assertEqual(rows[1].status, "SKIP")

    def test_build_report_with_fake_rows(self) -> None:
        rows = [
            smoke_reporter.SmokeRow(
                scene="ur5_2f_test",
                status="PASS",
                markers_found="Generated workcell context for scene",
                notes="Readiness markers detected",
                log_path="/tmp/ur5_2f_test.log",
            ),
            smoke_reporter.SmokeRow(
                scene="suction_test",
                status="FAIL",
                markers_found="",
                notes="Timeout",
                log_path="/tmp/suction_test.log",
            ),
        ]

        report_text = smoke_reporter.build_report(rows, Path("/tmp/results.tsv"))

        self.assertIn("Latest Smoke Launch Report", report_text)
        self.assertIn("| `ur5_2f_test` | **PASS**", report_text)
        self.assertIn("| `suction_test` | **FAIL**", report_text)
        self.assertIn("- PASS: 1", report_text)
        self.assertIn("- FAIL: 1", report_text)


if __name__ == "__main__":
    unittest.main()
