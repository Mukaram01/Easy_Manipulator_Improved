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
FIXTURES = REPO_ROOT / "tests" / "fixtures"
CAP_DIR = FIXTURES / "capabilities"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


validator = _load("validate_cell_definition_cap", REPO_ROOT / "scripts" / "validate_cell_definition.py")
scene_gen = _load("scene_gen_cap", REPO_ROOT / "scripts" / "generate_scene_from_cell_definition.py")
project_gen = REPO_ROOT / "scripts" / "create_workcell_project.py"
wizard = REPO_ROOT / "scripts" / "create_cell_definition_wizard.py"


class CellCapabilityIntegrationTests(unittest.TestCase):
    def test_valid_capability_cell_definition_passes(self) -> None:
        fixture = FIXTURES / "cell_definition_pick_place_with_capabilities.yaml"
        loaded, parser, notes = validator.load_yaml(fixture)
        summary = validator.validate_cell_definition(loaded, fixture, parser, notes, capabilities_dir=CAP_DIR)
        self.assertTrue(summary.ok)
        self.assertIn("robot", summary.capability_summary["resolved"])

    def test_existing_fixture_without_capabilities_still_passes(self) -> None:
        fixture = FIXTURES / "cell_definition_pick_place.yaml"
        loaded, parser, notes = validator.load_yaml(fixture)
        summary = validator.validate_cell_definition(loaded, fixture, parser, notes, capabilities_dir=CAP_DIR)
        self.assertTrue(summary.ok)

    def test_unknown_capability_warn_default_fail_strict(self) -> None:
        fixture = FIXTURES / "cell_definition_unknown_capability_warn.yaml"
        loaded, parser, notes = validator.load_yaml(fixture)
        warn_summary = validator.validate_cell_definition(loaded, fixture, parser, notes, capabilities_dir=CAP_DIR)
        strict_summary = validator.validate_cell_definition(loaded, fixture, parser, notes, strict=True, capabilities_dir=CAP_DIR)
        self.assertTrue(warn_summary.ok)
        self.assertTrue(warn_summary.warnings)
        self.assertFalse(strict_summary.ok)

    def test_incompatible_task_ee_robot_fails(self) -> None:
        fixture = FIXTURES / "cell_definition_incompatible_capability_fail.yaml"
        loaded, parser, notes = validator.load_yaml(fixture)
        summary = validator.validate_cell_definition(loaded, fixture, parser, notes, capabilities_dir=CAP_DIR)
        self.assertFalse(summary.ok)
        self.assertIn("magnetic", " ".join(summary.errors).lower())

    def test_generator_preserves_capability_ids(self) -> None:
        fixture = FIXTURES / "cell_definition_colour_sorting_with_capabilities.yaml"
        loaded, parser, notes = validator.load_yaml(fixture)
        summary = validator.validate_cell_definition(loaded, fixture, parser, notes, capabilities_dir=CAP_DIR)
        manifest = scene_gen.build_scene_manifest(loaded, summary.capability_summary)
        self.assertIn("capabilities", manifest)
        self.assertEqual(manifest["capabilities"]["robot"], "ur5")

    def test_validator_json_includes_capability_summary(self) -> None:
        proc = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts" / "validate_cell_definition.py"),
                str(FIXTURES / "cell_definition_pick_place_with_capabilities.yaml"),
                "--json",
                "--capabilities-dir",
                str(CAP_DIR),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertIn("capabilities", payload)
        self.assertIn("resolved", payload["capabilities"])

    def test_wizard_list_and_select_capabilities(self) -> None:
        list_proc = subprocess.run(
            [sys.executable, str(wizard), "--list-capabilities", "--capabilities-dir", str(CAP_DIR)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(list_proc.returncode, 0)
        self.assertIn("ur5", list_proc.stdout)

        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "cap_wizard.yaml"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(wizard),
                    "--template",
                    "pick_place",
                    "--cell-name",
                    "Wizard Caps",
                    "--cell-id",
                    "wizard_caps",
                    "--robot",
                    "ur5",
                    "--end-effector",
                    "robotiq_2f",
                    "--camera",
                    "realsense_d435i",
                    "--robot-capability",
                    "ur5",
                    "--end-effector-capability",
                    "robotiq_2f_85",
                    "--sensor-capability",
                    "intel_realsense_d435i",
                    "--task-capability",
                    "pick_place",
                    "--asset-capability",
                    "table_standard_1200",
                    "--output",
                    str(out),
                    "--force",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            text = out.read_text(encoding="utf-8")
            self.assertIn("capability: ur5", text)

    def test_project_manifest_records_capabilities(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            proc = subprocess.run(
                [
                    sys.executable,
                    str(project_gen),
                    "--cell-definition",
                    str(FIXTURES / "cell_definition_pick_place_with_capabilities.yaml"),
                    "--output-dir",
                    tmpdir,
                    "--force",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            manifest = json.loads((Path(tmpdir) / "ur5_pick_place_caps" / "project_manifest.json").read_text(encoding="utf-8"))
            self.assertIn("capabilities", manifest)
            self.assertEqual(manifest["capabilities"]["robot"], "ur5")


if __name__ == "__main__":
    unittest.main()
