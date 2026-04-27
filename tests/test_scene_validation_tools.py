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


if __name__ == "__main__":
    unittest.main()
