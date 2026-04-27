#!/usr/bin/env python3
"""Dependency-light tests for scene validation/report scripts."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures"


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
task_recipe_reporter = _load_module(
    "generate_task_recipe_report", REPO_ROOT / "scripts" / "generate_task_recipe_report.py"
)
task_recipe_dry_run = _load_module("dry_run_task_recipe", REPO_ROOT / "scripts" / "dry_run_task_recipe.py")
task_recipe_dry_run_reporter = _load_module(
    "generate_task_recipe_dry_run_report",
    REPO_ROOT / "scripts" / "generate_task_recipe_dry_run_report.py",
)
task_execution_plan = _load_module(
    "generate_task_execution_plan", REPO_ROOT / "scripts" / "generate_task_execution_plan.py"
)
task_execution_plan_reporter = _load_module(
    "generate_task_execution_plan_report", REPO_ROOT / "scripts" / "generate_task_execution_plan_report.py"
)
workcell_bundle_exporter = _load_module(
    "export_workcell_bundle", REPO_ROOT / "scripts" / "export_workcell_bundle.py"
)
workcell_bundle_inspector = _load_module(
    "inspect_workcell_bundle", REPO_ROOT / "scripts" / "inspect_workcell_bundle.py"
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
  allowed_touch_links: {name: link_a}
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

    def test_validate_scene_manifest_path_accepts_direct_file(self) -> None:
        result, exit_code = validator.validate_scene_manifest_path(str(FIXTURES / "generated_scene_manifest.yaml"))
        self.assertEqual(exit_code, 0)
        self.assertTrue(result.ok)
        self.assertEqual(result.manifest_path, str(FIXTURES / "generated_scene_manifest.yaml"))


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


class TaskRecipeValidationTests(unittest.TestCase):
    def test_valid_task_recipe_block(self) -> None:
        manifest = {
            "task_recipe": {
                "id": "colour_sort_demo",
                "name": "Colour Sort Demo",
                "type": "sort",
                "enabled": True,
                "pick": {"object_source": "perception", "allowed_grasp_methods": ["finger"]},
                "decision_rules": [
                    {"id": "rule_a", "when": {"attribute": "colour", "equals": "red"}, "destination": "bin_a"},
                    {"id": "default", "when": {"default": True}, "destination": "reject_bin"},
                ],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    },
                    {
                        "id": "reject_bin",
                        "frame_id": "world",
                        "pose_xyz": [0.2, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "reject",
                    },
                ],
            }
        }
        status, notes = validator.validate_task_recipe_block(manifest)
        self.assertEqual(status, "PASS")
        self.assertIn("present and valid", " ".join(notes))

    def test_missing_task_recipe_warns(self) -> None:
        status, notes = validator.validate_task_recipe_block({})
        self.assertEqual(status, "WARN")
        self.assertIn("not defined", " ".join(notes))

    def test_invalid_destination_reference_fails(self) -> None:
        manifest = {
            "task_recipe": {
                "id": "test",
                "name": "Test",
                "type": "sort",
                "enabled": True,
                "decision_rules": [
                    {
                        "id": "bad_ref",
                        "when": {"attribute": "colour", "equals": "red"},
                        "destination": "missing_bin",
                    }
                ],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    }
                ],
            }
        }
        status, notes = validator.validate_task_recipe_block(manifest)
        self.assertEqual(status, "FAIL")
        self.assertIn("does not match any task_recipe.destinations.id", " ".join(notes))

    def test_invalid_destination_pose_length_fails(self) -> None:
        manifest = {
            "task_recipe": {
                "id": "test",
                "name": "Test",
                "type": "sort",
                "enabled": True,
                "decision_rules": [{"id": "default", "when": {"default": True}, "destination": "bin_a"}],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    }
                ],
            }
        }
        status, notes = validator.validate_task_recipe_block(manifest)
        self.assertEqual(status, "FAIL")
        self.assertIn("pose_xyz", " ".join(notes))


class TaskRecipeDryRunTests(unittest.TestCase):
    def _base_manifest(self) -> dict[str, object]:
        return {
            "self_test": {
                "enabled": True,
                "object": {
                    "id": "commissioning_box",
                    "shape": "box",
                    "dimensions": [0.05, 0.05, 0.05],
                    "frame_id": "world",
                    "pose_xyz": [0.45, 0.0, 0.08],
                    "pose_rpy": [0.0, 0.0, 0.0],
                    "attributes": {"class": "part", "colour": "red", "shape": "box"},
                },
            },
            "task_recipe": {
                "id": "colour_sort",
                "name": "Colour Sort",
                "type": "sort",
                "enabled": True,
                "decision_rules": [
                    {"id": "red_to_bin_a", "when": {"attribute": "colour", "equals": "red"}, "destination": "bin_a"},
                    {"id": "default_reject", "when": {"default": True}, "destination": "reject_bin"},
                ],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    },
                    {
                        "id": "reject_bin",
                        "frame_id": "world",
                        "pose_xyz": [0.2, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "reject",
                    },
                ],
            },
        }

    def test_valid_self_test_and_task_recipe_resolves_pass(self) -> None:
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(self._base_manifest()))
        self.assertEqual(row.status, "PASS")
        self.assertEqual(row.selected_destination_id, "bin_a")

    def test_missing_task_recipe_warns_not_fail(self) -> None:
        manifest = self._base_manifest()
        manifest.pop("task_recipe")
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(manifest))
        self.assertEqual(row.status, "WARN")

    def test_missing_self_test_skips_not_fail(self) -> None:
        manifest = self._base_manifest()
        manifest.pop("self_test")
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(manifest))
        self.assertEqual(row.status, "SKIP")

    def test_attribute_equals_rule_resolves_destination(self) -> None:
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(self._base_manifest()))
        self.assertEqual(row.matched_rule_id, "red_to_bin_a")
        self.assertEqual(row.selected_destination_id, "bin_a")

    def test_default_rule_resolves_when_no_match(self) -> None:
        manifest = self._base_manifest()
        manifest["self_test"]["object"]["attributes"]["colour"] = "green"
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(manifest))
        self.assertEqual(row.status, "PASS")
        self.assertEqual(row.matched_rule_id, "default_reject")
        self.assertEqual(row.selected_destination_id, "reject_bin")

    def test_missing_destination_reference_fails(self) -> None:
        manifest = self._base_manifest()
        manifest["task_recipe"]["decision_rules"][0]["destination"] = "missing_bin"
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(manifest))
        self.assertEqual(row.status, "FAIL")
        self.assertIn("does not match any task_recipe.destinations.id", " ".join(row.notes))

    def test_malformed_destination_pose_fails(self) -> None:
        manifest = self._base_manifest()
        manifest["task_recipe"]["destinations"][0]["pose_xyz"] = [0.3, 0.0]
        row = task_recipe_dry_run.evaluate_scene("scene_a", self._write_manifest(manifest))
        self.assertEqual(row.status, "FAIL")
        self.assertIn("pose_xyz", " ".join(row.notes))

    def test_report_generation_with_fake_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_root = Path(tmpdir)
            (tmp_root / "scenes" / "scene_a").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_a" / "scene_manifest.yaml").write_text(
                "self_test:\n"
                "  enabled: true\n"
                "  object:\n"
                "    id: commissioning_box\n"
                "    shape: box\n"
                "    dimensions: [0.05, 0.05, 0.05]\n"
                "    frame_id: world\n"
                "    pose_xyz: [0.45, 0.0, 0.08]\n"
                "    pose_rpy: [0.0, 0.0, 0.0]\n"
                "    attributes:\n"
                "      class: part\n"
                "      colour: red\n"
                "task_recipe:\n"
                "  id: colour_sort\n"
                "  name: Colour Sort\n"
                "  type: sort\n"
                "  enabled: true\n"
                "  decision_rules:\n"
                "    - id: red_to_bin_a\n"
                "      when:\n"
                "        attribute: colour\n"
                "        equals: red\n"
                "      destination: bin_a\n"
                "  destinations:\n"
                "    - id: bin_a\n"
                "      frame_id: world\n"
                "      pose_xyz: [0.3, 0.0, 0.1]\n"
                "      pose_rpy: [0.0, 0.0, 0.0]\n"
                "      action: place\n",
                encoding="utf-8",
            )
            with mock.patch.object(task_recipe_dry_run_reporter, "REPO_ROOT", tmp_root):
                with mock.patch.object(task_recipe_dry_run, "REPO_ROOT", tmp_root):
                    with mock.patch.object(
                        task_recipe_dry_run_reporter,
                        "REPORT_PATH",
                        tmp_root / "docs" / "manuals" / "dry_run_report.md",
                    ):
                        discovered = task_recipe_dry_run.discover_scene_manifests()
                        rows = [task_recipe_dry_run.evaluate_scene(name, path) for name, path in discovered]
                        report_text = task_recipe_dry_run_reporter.build_report(rows)

        self.assertIn("`scene_a` | **PASS**", report_text)

    def test_fallback_parser_parses_self_test_attributes(self) -> None:
        manifest_text = (
            "self_test:\n"
            "  enabled: true\n"
            "  object:\n"
            "    id: commissioning_box\n"
            "    shape: box\n"
            "    dimensions: [0.05, 0.05, 0.05]\n"
            "    frame_id: world\n"
            "    pose_xyz: [0.45, 0.0, 0.08]\n"
            "    pose_rpy: [0.0, 0.0, 0.0]\n"
            "    attributes:\n"
            "      class: part\n"
            "      colour: red\n"
            "      shape: box\n"
        )
        parsed = validator.parse_manifest_yaml(manifest_text)
        self.assertEqual(parsed["self_test"]["object"]["attributes"]["class"], "part")
        self.assertEqual(parsed["self_test"]["object"]["attributes"]["colour"], "red")

    def _write_manifest(self, manifest: dict[str, object]) -> Path:
        def _to_yaml(value: object, indent: int = 0) -> str:
            def _scalar_text(scalar: object) -> str:
                if isinstance(scalar, bool):
                    return "true" if scalar else "false"
                return str(scalar)

            prefix = " " * indent
            if isinstance(value, dict):
                lines: list[str] = []
                for key, child in value.items():
                    if isinstance(child, (dict, list)):
                        lines.append(f"{prefix}{key}:")
                        lines.append(_to_yaml(child, indent + 2))
                    else:
                        lines.append(f"{prefix}{key}: {_scalar_text(child)}")
                return "\n".join(lines)
            if isinstance(value, list):
                lines = []
                for child in value:
                    if isinstance(child, dict):
                        first = True
                        for key, nested in child.items():
                            if first:
                                if isinstance(nested, (dict, list)):
                                    lines.append(f"{prefix}- {key}:")
                                    lines.append(_to_yaml(nested, indent + 4))
                                else:
                                    lines.append(f"{prefix}- {key}: {_scalar_text(nested)}")
                                first = False
                                continue
                            if isinstance(nested, (dict, list)):
                                lines.append(f"{prefix}  {key}:")
                                lines.append(_to_yaml(nested, indent + 4))
                            else:
                                lines.append(f"{prefix}  {key}: {_scalar_text(nested)}")
                    elif isinstance(child, list):
                        lines.append(f"{prefix}-")
                        lines.append(_to_yaml(child, indent + 2))
                    else:
                        lines.append(f"{prefix}- {_scalar_text(child)}")
                return "\n".join(lines)
            return f"{prefix}{_scalar_text(value)}"

        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            if validator._pyyaml is not None:
                validator._pyyaml.safe_dump(manifest, handle, sort_keys=False)
            else:
                handle.write(_to_yaml(manifest))
            return Path(handle.name)

    def test_invalid_task_type_fails(self) -> None:
        manifest = {
            "task_recipe": {
                "id": "test",
                "name": "Test",
                "type": "unknown",
                "enabled": True,
                "decision_rules": [{"id": "default", "when": {"default": True}, "destination": "bin_a"}],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    }
                ],
            }
        }
        status, _ = validator.validate_task_recipe_block(manifest)
        self.assertEqual(status, "FAIL")

    def test_task_recipe_report_generation_with_fake_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_root = Path(tmpdir)
            (tmp_root / "scenes" / "scene_a").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_b").mkdir(parents=True)
            (tmp_root / "scenes" / "scene_a" / "scene_manifest.yaml").write_text(
                "task_recipe:\n"
                "  id: colour_sort_demo\n"
                "  name: Colour Sort Demo\n"
                "  type: sort\n"
                "  enabled: true\n"
                "  decision_rules:\n"
                "    - id: default_rule\n"
                "      when:\n"
                "        default: true\n"
                "      destination: bin_a\n"
                "  destinations:\n"
                "    - id: bin_a\n"
                "      frame_id: world\n"
                "      pose_xyz: [0.3, 0.0, 0.1]\n"
                "      pose_rpy: [0.0, 0.0, 0.0]\n"
                "      action: place\n",
                encoding="utf-8",
            )
            (tmp_root / "scenes" / "scene_b" / "workcell.yaml").write_text(
                "scene:\n  name: scene_b\n",
                encoding="utf-8",
            )

            with mock.patch.object(task_recipe_reporter, "REPO_ROOT", tmp_root):
                with mock.patch.object(
                    task_recipe_reporter,
                    "REPORT_PATH",
                    tmp_root / "docs" / "manuals" / "task_recipe_report.md",
                ):
                    discovered = task_recipe_reporter.discover_scene_manifests()
                    rows = [task_recipe_reporter.evaluate_scene(name, path) for name, path in discovered]
                    report_text = task_recipe_reporter.build_report(rows)

        self.assertEqual(len(rows), 2)
        self.assertIn("`scene_a`", report_text)
        self.assertIn("**PASS**", report_text)
        self.assertIn("**WARN**", report_text)


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


class TaskExecutionPlanTests(unittest.TestCase):
    def _base_manifest(self, ee_type: str = "finger", ee_brand: str = "robotiq_2f") -> dict[str, object]:
        return {
            "robot": {
                "planning_group": "manipulator",
                "base_frame": "world",
                "ee_link": "tool0",
            },
            "end_effector": {
                "type": ee_type,
                "brand": ee_brand,
                "grasp_frame": "ee_palm",
            },
            "self_test": {
                "enabled": True,
                "object": {
                    "id": "commissioning_box",
                    "shape": "box",
                    "dimensions": [0.05, 0.05, 0.05],
                    "frame_id": "world",
                    "pose_xyz": [0.45, 0.0, 0.08],
                    "pose_rpy": [0.0, 0.0, 0.0],
                    "attributes": {"class": "part", "colour": "red", "shape": "box"},
                },
            },
            "task_recipe": {
                "id": "colour_sort_demo",
                "name": "Colour Sort",
                "type": "sort",
                "enabled": True,
                "decision_rules": [
                    {
                        "id": "red_to_bin_a",
                        "when": {"attribute": "colour", "equals": "red"},
                        "destination": "bin_a",
                    },
                    {"id": "default_reject", "when": {"default": True}, "destination": "reject_bin"},
                ],
                "destinations": [
                    {
                        "id": "bin_a",
                        "frame_id": "world",
                        "pose_xyz": [0.3, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "place",
                    },
                    {
                        "id": "reject_bin",
                        "frame_id": "world",
                        "pose_xyz": [0.2, 0.0, 0.1],
                        "pose_rpy": [0.0, 0.0, 0.0],
                        "action": "reject",
                    },
                ],
            },
        }

    def _write_manifest(self, root: Path, scene: str, manifest: dict[str, object]) -> Path:
        def _to_yaml(value: object, indent: int = 0) -> str:
            def _scalar_text(scalar: object) -> str:
                if isinstance(scalar, bool):
                    return "true" if scalar else "false"
                return str(scalar)

            prefix = " " * indent
            if isinstance(value, dict):
                lines: list[str] = []
                for key, child in value.items():
                    if isinstance(child, (dict, list)):
                        lines.append(f"{prefix}{key}:")
                        lines.append(_to_yaml(child, indent + 2))
                    else:
                        lines.append(f"{prefix}{key}: {_scalar_text(child)}")
                return "\n".join(lines)
            if isinstance(value, list):
                lines = []
                for child in value:
                    if isinstance(child, dict):
                        first = True
                        for key, nested in child.items():
                            if first:
                                if isinstance(nested, (dict, list)):
                                    lines.append(f"{prefix}- {key}:")
                                    lines.append(_to_yaml(nested, indent + 4))
                                else:
                                    lines.append(f"{prefix}- {key}: {_scalar_text(nested)}")
                                first = False
                                continue
                            if isinstance(nested, (dict, list)):
                                lines.append(f"{prefix}  {key}:")
                                lines.append(_to_yaml(nested, indent + 4))
                            else:
                                lines.append(f"{prefix}  {key}: {_scalar_text(nested)}")
                    else:
                        lines.append(f"{prefix}- {_scalar_text(child)}")
                return "\n".join(lines)
            return f"{prefix}{_scalar_text(value)}"

        scene_dir = root / "scenes" / scene
        scene_dir.mkdir(parents=True, exist_ok=True)
        path = scene_dir / "scene_manifest.yaml"
        if validator._pyyaml is not None:
            path.write_text(validator._pyyaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        else:
            path.write_text(_to_yaml(manifest), encoding="utf-8")
        return path

    def _plan_actions(self, plan: dict[str, object]) -> tuple[str, str]:
        close_action = next(step["action"] for step in plan["steps"] if step["id"] == "close_end_effector")
        open_action = next(step["action"] for step in plan["steps"] if step["id"] == "release_object")
        return close_action, open_action

    def test_valid_dry_run_pass_produces_plan_steps(self) -> None:
        dry_row = task_recipe_dry_run.DryRunResult(
            scene="scene_a",
            status="PASS",
            recipe_id="colour_sort_demo",
            task_type="sort",
            object_id="commissioning_box",
            object_attributes={"colour": "red"},
            matched_rule_id="red_to_bin_a",
            selected_destination_id="bin_a",
            selected_action="place",
            notes=[],
        )
        plan = task_execution_plan.build_execution_plan("scene_a", self._base_manifest(), dry_row)
        step_ids = [step["id"] for step in plan["steps"]]
        self.assertIn("acquire_object", step_ids)
        self.assertIn("close_end_effector", step_ids)
        self.assertIn("release_object", step_ids)
        self.assertIn("return_home", step_ids)

    def test_robotiq_scene_uses_gripper_labels(self) -> None:
        dry_row = task_recipe_dry_run.DryRunResult("scene_a", "PASS", "id", "sort", "o", {}, "r", "bin_a", "place", [])
        plan = task_execution_plan.build_execution_plan("scene_a", self._base_manifest("finger", "robotiq_2f"), dry_row)
        self.assertEqual(self._plan_actions(plan), ("close_gripper", "open_gripper"))

    def test_suction_scene_uses_suction_labels(self) -> None:
        dry_row = task_recipe_dry_run.DryRunResult("scene_a", "PASS", "id", "sort", "o", {}, "r", "bin_a", "place", [])
        plan = task_execution_plan.build_execution_plan("scene_a", self._base_manifest("suction", "airpick4"), dry_row)
        self.assertEqual(self._plan_actions(plan), ("activate_suction", "deactivate_suction"))

    def test_unknown_end_effector_uses_generic_labels(self) -> None:
        dry_row = task_recipe_dry_run.DryRunResult("scene_a", "PASS", "id", "sort", "o", {}, "r", "bin_a", "place", [])
        plan = task_execution_plan.build_execution_plan("scene_a", self._base_manifest("none", "custom_ee"), dry_row)
        self.assertEqual(self._plan_actions(plan), ("engage_end_effector", "release_end_effector"))

    def test_missing_task_recipe_warn_skip_no_hard_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._base_manifest()
            manifest.pop("task_recipe")
            path = self._write_manifest(root, "scene_warn", manifest)
            with mock.patch.object(task_execution_plan.dry_run, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan, "OUTPUT_DIR", root / "docs" / "manuals" / "generated_execution_plans"
            ):
                row = task_execution_plan.evaluate_scene("scene_warn", path)
        self.assertEqual(row.status, "WARN")
        self.assertIsNone(row.markdown_path)

    def test_missing_self_test_skip_no_hard_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._base_manifest()
            manifest.pop("self_test")
            path = self._write_manifest(root, "scene_skip", manifest)
            with mock.patch.object(task_execution_plan.dry_run, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan, "OUTPUT_DIR", root / "docs" / "manuals" / "generated_execution_plans"
            ):
                row = task_execution_plan.evaluate_scene("scene_skip", path)
        self.assertEqual(row.status, "SKIP")
        self.assertIsNone(row.json_path)

    def test_dry_run_fail_prevents_plan_generation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest = self._base_manifest()
            manifest["task_recipe"]["decision_rules"] = []
            path = self._write_manifest(root, "scene_fail", manifest)
            with mock.patch.object(task_execution_plan.dry_run, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan, "OUTPUT_DIR", root / "docs" / "manuals" / "generated_execution_plans"
            ):
                row = task_execution_plan.evaluate_scene("scene_fail", path)
        self.assertEqual(row.status, "FAIL")
        self.assertEqual(row.steps_count, 0)

    def test_generated_markdown_and_json_paths_created(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            path = self._write_manifest(root, "scene_pass", self._base_manifest())
            out_dir = root / "docs" / "manuals" / "generated_execution_plans"
            with mock.patch.object(task_execution_plan.dry_run, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan, "OUTPUT_DIR", out_dir
            ):
                row = task_execution_plan.evaluate_scene("scene_pass", path)
                self.assertEqual(row.status, "PASS")
                self.assertTrue(row.markdown_path and row.markdown_path.is_file())
                self.assertTrue(row.json_path and row.json_path.is_file())

    def test_summary_report_generation_with_fake_manifests(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            self._write_manifest(root, "scene_a", self._base_manifest())
            with mock.patch.object(task_execution_plan_reporter, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan_reporter, "REPORT_PATH", root / "docs" / "manuals" / "latest_task_execution_plan_report.md"
            ), mock.patch.object(task_execution_plan_reporter.plan_generator.dry_run, "REPO_ROOT", root), mock.patch.object(
                task_execution_plan_reporter.plan_generator,
                "OUTPUT_DIR",
                root / "docs" / "manuals" / "generated_execution_plans",
            ):
                discovered = task_execution_plan_reporter.plan_generator.dry_run.discover_scene_manifests()
                rows = [task_execution_plan_reporter.plan_generator.evaluate_scene(name, path) for name, path in discovered]
                report_text = task_execution_plan_reporter.build_report(rows)
        self.assertIn("`scene_a` | **PASS**", report_text)
        self.assertIn("deterministic operator-readable job sequence", report_text)


class WorkcellBundleTests(unittest.TestCase):
    def _manifest_text(self) -> str:
        return (
            "scene:\n"
            "  name: fake_scene\n"
            "robot:\n"
            "  model: ur5\n"
            "  planning_group: manipulator\n"
            "  base_frame: base_link\n"
            "  ee_link: tool0\n"
            "end_effector:\n"
            "  type: finger\n"
            "  brand: robotiq_2f\n"
            "  grasp_frame: tool0\n"
            "self_test:\n"
            "  enabled: true\n"
            "  object:\n"
            "    id: commissioning_box\n"
            "    shape: box\n"
            "    dimensions: [0.05, 0.05, 0.05]\n"
            "    frame_id: world\n"
            "    pose_xyz: [0.45, 0.0, 0.08]\n"
            "    pose_rpy: [0.0, 0.0, 0.0]\n"
            "task_recipe:\n"
            "  id: colour_sort_demo\n"
            "  name: Colour Sort Demo\n"
            "  type: sort\n"
            "  enabled: true\n"
            "  decision_rules:\n"
            "    - id: red_to_bin_a\n"
            "      when:\n"
            "        attribute: colour\n"
            "        equals: red\n"
            "      destination: bin_a\n"
            "    - id: default\n"
            "      when:\n"
            "        default: true\n"
            "      destination: reject_bin\n"
            "  destinations:\n"
            "    - id: bin_a\n"
            "      frame_id: world\n"
            "      pose_xyz: [0.20, 0.0, 0.12]\n"
            "      pose_rpy: [0.0, 0.0, 0.0]\n"
            "      action: place\n"
            "    - id: reject_bin\n"
            "      frame_id: world\n"
            "      pose_xyz: [0.15, 0.0, 0.12]\n"
            "      pose_rpy: [0.0, 0.0, 0.0]\n"
            "      action: reject\n"
        )

    def _write_scene(self, root: Path, name: str = "fake_scene") -> Path:
        manifest_path = root / "scenes" / name / "scene_manifest.yaml"
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        manifest_path.write_text(self._manifest_text().replace("fake_scene", name), encoding="utf-8")
        return manifest_path

    def _configure_bundle_modules(self, root: Path) -> None:
        workcell_bundle_exporter.REPO_ROOT = root
        workcell_bundle_exporter.DEFAULT_OUTPUT_DIR = root / "dist" / "workcell_bundles"
        workcell_bundle_exporter.REPORTS_DIR = root / "docs" / "manuals"
        workcell_bundle_exporter.PLAN_OUTPUT_DIR = root / "docs" / "manuals" / "generated_execution_plans"
        workcell_bundle_exporter.dry_run.REPO_ROOT = root
        workcell_bundle_exporter.plan_generator.REPO_ROOT = root
        workcell_bundle_exporter.plan_generator.dry_run.REPO_ROOT = root
        workcell_bundle_exporter.plan_generator.OUTPUT_DIR = root / "docs" / "manuals" / "generated_execution_plans"
        workcell_bundle_exporter.self_test_reporter.REPO_ROOT = root
        workcell_bundle_exporter.task_recipe_reporter.REPO_ROOT = root

    def test_bundle_export_and_manifest_fields(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                "fake_scene", manifest_path, root / "dist" / "workcell_bundles", zip_output=False, force=True
            )
            self.assertTrue((bundle_dir / "README.md").is_file())
            self.assertTrue((bundle_dir / "operator_checklist.md").is_file())
            manifest = json.loads((bundle_dir / "bundle_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["bundle_schema_version"], "1.0")
            self.assertEqual(manifest["scene"], "fake_scene")
            self.assertTrue(manifest["offline_only"])
            self.assertIn("files", manifest)

    def test_operator_checklist_sections_present(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                "fake_scene", manifest_path, root / "dist" / "workcell_bundles", zip_output=False, force=True
            )
            checklist = (bundle_dir / "operator_checklist.md").read_text(encoding="utf-8")
            self.assertIn("## Offline checks", checklist)
            self.assertIn("## Simulation checks", checklist)
            self.assertIn("## Physical cell checks", checklist)

    def test_missing_optional_reports_warn_not_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                "fake_scene", manifest_path, root / "dist" / "workcell_bundles", zip_output=False, force=True
            )
            manifest = json.loads((bundle_dir / "bundle_manifest.json").read_text(encoding="utf-8"))
            self.assertTrue(any("Optional report missing" in warning for warning in manifest["warnings"]))

    def test_inspector_pass_and_checksum_mismatch_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                "fake_scene", manifest_path, root / "dist" / "workcell_bundles", zip_output=False, force=True
            )
            status, _ = workcell_bundle_inspector.inspect_bundle(bundle_dir)
            self.assertEqual(status, "PASS")

            readme = bundle_dir / "README.md"
            readme.write_text(readme.read_text(encoding="utf-8") + "\nTamper\n", encoding="utf-8")
            status_after, notes = workcell_bundle_inspector.inspect_bundle(bundle_dir)
            self.assertEqual(status_after, "FAIL")
            self.assertTrue(any("Checksum mismatch" in note for note in notes))

    def test_zip_export_and_force_flag(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            out_dir = root / "dist" / "workcell_bundles"
            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                "fake_scene", manifest_path, out_dir, zip_output=True, force=True
            )
            zip_path = out_dir / "fake_scene.zip"
            self.assertTrue(zip_path.is_file())

            status, _ = workcell_bundle_inspector.inspect_bundle(zip_path)
            self.assertEqual(status, "PASS")

            with self.assertRaises(RuntimeError):
                workcell_bundle_exporter._zip_bundle("fake_scene", bundle_dir, out_dir, force=False)

            zip_forced = workcell_bundle_exporter._zip_bundle("fake_scene", bundle_dir, out_dir, force=True)
            self.assertTrue(zip_forced.is_file())

    def test_missing_execution_plan_with_dry_run_pass_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            manifest_path = self._write_scene(root)
            self._configure_bundle_modules(root)
            out_dir = root / "dist" / "workcell_bundles"

            with mock.patch.object(workcell_bundle_exporter.plan_generator, "evaluate_scene") as mocked_eval:
                mocked_eval.return_value = task_execution_plan.PlanResult(
                    scene="fake_scene",
                    status="PASS",
                    task_recipe_id="id",
                    task_type="sort",
                    matched_rule_id="rule",
                    destination_id="bin_a",
                    markdown_path=None,
                    json_path=None,
                    steps_count=0,
                    notes=["forced failure"],
                )
                with self.assertRaises(RuntimeError):
                    workcell_bundle_exporter.export_scene(
                        "fake_scene", manifest_path, out_dir, zip_output=False, force=True
                    )


class GeneratedSceneFixtureTests(unittest.TestCase):
    def _fixture_path(self) -> Path:
        return REPO_ROOT / "tests" / "fixtures" / "generated_scene_manifest.yaml"

    def _load_fixture_manifest(self) -> dict[str, object]:
        manifest, _, _ = validator._read_manifest(str(self._fixture_path()))
        return manifest

    def test_generated_manifest_fixture_validates(self) -> None:
        manifest = self._load_fixture_manifest()
        status, notes = validator.validate_task_recipe_block(manifest)
        self.assertIn(status, {"PASS", "WARN"})
        self.assertTrue(any("present and valid" in note for note in notes))

    def test_generated_manifest_has_required_generated_blocks(self) -> None:
        manifest = self._load_fixture_manifest()
        self.assertIn("self_test", manifest)
        self.assertIn("task_recipe", manifest)
        self.assertIn("home_return", manifest)
        self.assertIn("safe_joint_state", manifest["home_return"])

    def test_empty_safe_joint_state_allowed_with_named_target(self) -> None:
        manifest = self._load_fixture_manifest()
        errors: list[str] = []
        warnings: list[str] = []
        validator._check_types(manifest, errors, warnings, "generated_demo_scene")
        self.assertFalse(errors)
        self.assertTrue(any("named target fallback" in warning for warning in warnings))

    def test_malformed_generated_destination_pose_fails(self) -> None:
        manifest = self._load_fixture_manifest()
        manifest["task_recipe"]["destinations"][0]["pose_xyz"] = [0.3, -0.3]
        status, notes = validator.validate_task_recipe_block(manifest)
        self.assertEqual(status, "FAIL")
        self.assertIn("pose_xyz", " ".join(notes))

    def test_generated_task_recipe_dry_run_does_not_crash(self) -> None:
        row = task_recipe_dry_run.evaluate_scene("generated_demo_scene", self._fixture_path())
        self.assertIn(row.status, {"PASS", "WARN"})

    def test_execution_plan_generator_handles_generated_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "plans"
            with mock.patch.object(task_execution_plan, "OUTPUT_DIR", output_dir):
                row = task_execution_plan.evaluate_scene("generated_demo_scene", self._fixture_path())
        self.assertIn(row.status, {"PASS", "WARN"})

    def test_bundle_exporter_packages_generated_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            scene_name = "generated_demo_scene"
            scene_dir = root / "scenes" / scene_name
            scene_dir.mkdir(parents=True, exist_ok=True)
            fixture_text = self._fixture_path().read_text(encoding="utf-8")
            manifest_path = scene_dir / "scene_manifest.yaml"
            manifest_path.write_text(fixture_text, encoding="utf-8")

            workcell_bundle_exporter.REPO_ROOT = root
            workcell_bundle_exporter.DEFAULT_OUTPUT_DIR = root / "dist" / "workcell_bundles"
            workcell_bundle_exporter.REPORTS_DIR = root / "docs" / "manuals"
            workcell_bundle_exporter.PLAN_OUTPUT_DIR = root / "docs" / "manuals" / "generated_execution_plans"
            workcell_bundle_exporter.dry_run.REPO_ROOT = root
            workcell_bundle_exporter.plan_generator.REPO_ROOT = root
            workcell_bundle_exporter.plan_generator.dry_run.REPO_ROOT = root
            workcell_bundle_exporter.plan_generator.OUTPUT_DIR = (
                root / "docs" / "manuals" / "generated_execution_plans"
            )
            workcell_bundle_exporter.self_test_reporter.REPO_ROOT = root
            workcell_bundle_exporter.task_recipe_reporter.REPO_ROOT = root

            bundle_dir, _ = workcell_bundle_exporter.export_scene(
                scene_name, manifest_path, root / "dist" / "workcell_bundles", zip_output=False, force=True
            )
            self.assertTrue((bundle_dir / "bundle_manifest.json").is_file())


if __name__ == "__main__":
    unittest.main()
