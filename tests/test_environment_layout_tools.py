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
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "environment_layouts"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


inventory = _load("inspect_asset_inventory", REPO_ROOT / "scripts" / "inspect_asset_inventory.py")
layout_validator = _load("validate_environment_layout", REPO_ROOT / "scripts" / "validate_environment_layout.py")
cell_validator = _load("validate_cell_definition_layout", REPO_ROOT / "scripts" / "validate_cell_definition.py")
project_gen = REPO_ROOT / "scripts" / "create_workcell_project.py"


class EnvironmentLayoutToolTests(unittest.TestCase):
    def test_asset_inventory_runs(self) -> None:
        payload = inventory.collect_inventory(REPO_ROOT)
        self.assertEqual(payload["schema_version"], "asset_inventory/v1")
        self.assertGreater(payload["totals"]["files"], 0)

    def test_valid_layout_passes(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "ur5_table_bins_existing_assets.layout.yaml")
        summary = layout_validator.validate_layout(loaded, FIXTURES / "ur5_table_bins_existing_assets.layout.yaml", parser, notes)
        self.assertTrue(summary.ok)

    def test_duplicate_asset_id_fails(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "invalid_duplicate_asset_id.layout.yaml")
        summary = layout_validator.validate_layout(loaded, FIXTURES / "invalid_duplicate_asset_id.layout.yaml", parser, notes)
        self.assertFalse(summary.ok)

    def test_missing_pose_fails(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "invalid_missing_pose.layout.yaml")
        summary = layout_validator.validate_layout(loaded, FIXTURES / "invalid_missing_pose.layout.yaml", parser, notes)
        self.assertFalse(summary.ok)

    def test_invalid_bounds_fail(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "invalid_bounds.layout.yaml")
        summary = layout_validator.validate_layout(loaded, FIXTURES / "invalid_bounds.layout.yaml", parser, notes)
        self.assertFalse(summary.ok)

    def test_package_uri_accepted(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "ur5_conveyor_sorting_existing_assets.layout.yaml")
        summary = layout_validator.validate_layout(loaded, FIXTURES / "ur5_conveyor_sorting_existing_assets.layout.yaml", parser, notes)
        self.assertTrue(summary.ok)

    def test_repo_relative_missing_path_warn_default_and_fail_strict(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "missing_path.layout.yaml"
            path.write_text(
                "schema_version: environment_layout/v1\n"
                "layout_id: missing_path\n"
                "assets:\n"
                "  - id: missing\n"
                "    source:\n"
                "      path: assets/environment/does_not_exist.stl\n"
                "    type: table\n"
                "    pose:\n"
                "      frame: world\n"
                "      xyz: [0.0, 0.0, 0.0]\n"
                "      rpy: [0.0, 0.0, 0.0]\n"
                "zones: []\n",
                encoding="utf-8",
            )
            loaded, parser, notes = layout_validator.load_layout(path)
            warn_summary = layout_validator.validate_layout(loaded, path, parser, notes, strict=False)
            strict_summary = layout_validator.validate_layout(loaded, path, parser, notes, strict=True)
            self.assertTrue(warn_summary.ok)
            self.assertTrue(warn_summary.warnings)
            self.assertFalse(strict_summary.ok)

    def test_unknown_asset_ref_warn_default_fail_strict(self) -> None:
        loaded, parser, notes = layout_validator.load_layout(FIXTURES / "unknown_asset_ref_warn.layout.yaml")
        warn_summary = layout_validator.validate_layout(loaded, FIXTURES / "unknown_asset_ref_warn.layout.yaml", parser, notes, strict=False)
        strict_summary = layout_validator.validate_layout(loaded, FIXTURES / "unknown_asset_ref_warn.layout.yaml", parser, notes, strict=True)
        self.assertTrue(warn_summary.ok)
        self.assertTrue(warn_summary.warnings)
        self.assertFalse(strict_summary.ok)

    def test_cell_definition_with_layout_validates(self) -> None:
        fixture = REPO_ROOT / "tests" / "fixtures" / "cell_definition_pick_place_with_layout.yaml"
        loaded, parser, notes = cell_validator.load_yaml(fixture)
        summary = cell_validator.validate_cell_definition(loaded, fixture, parser, notes)
        self.assertTrue(summary.ok)
        self.assertIn("path", summary.environment_layout_summary)

    def test_generated_manifests_preserve_environment_layout_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            proc = subprocess.run(
                [
                    sys.executable,
                    str(project_gen),
                    "--cell-definition",
                    str(REPO_ROOT / "tests" / "fixtures" / "cell_definition_pick_place_with_layout.yaml"),
                    "--output-dir",
                    tmpdir,
                    "--force",
                    "--quiet",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            manifest_path = Path(tmpdir) / "ur5_pick_place_layout" / "project_manifest.json"
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertIn("environment_layout", payload)
            self.assertTrue(payload["environment_layout"])

    def test_existing_cell_without_layout_still_passes(self) -> None:
        fixture = REPO_ROOT / "tests" / "fixtures" / "cell_definition_pick_place.yaml"
        loaded, parser, notes = cell_validator.load_yaml(fixture)
        summary = cell_validator.validate_cell_definition(loaded, fixture, parser, notes)
        self.assertTrue(summary.ok)


if __name__ == "__main__":
    unittest.main()
