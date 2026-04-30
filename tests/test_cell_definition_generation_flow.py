#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.validate_cell_definition import load_yaml, validate_cell_definition
from scripts.workcell_discovery import discover_all

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO = REPO_ROOT / "cell_definitions/demo_ur5_sorting_cell.yaml"


class CellDefinitionFlowTests(unittest.TestCase):
    def test_demo_cell_definition_validates(self):
        loaded, parser, notes = load_yaml(DEMO)
        summary = validate_cell_definition(loaded, DEMO, parser, notes)
        self.assertTrue(summary.ok, msg="\n".join(summary.errors + summary.warnings))

    def test_invalid_dimensions_fail(self):
        loaded, parser, notes = load_yaml(DEMO)
        loaded["objects"][0]["dimensions"] = [0.2, -0.1, 0.1]
        summary = validate_cell_definition(loaded, DEMO, parser, notes)
        self.assertFalse(summary.ok)
        self.assertTrue(any("positive number" in e for e in summary.errors))

    def test_duplicate_object_id_fails(self):
        loaded, parser, notes = load_yaml(DEMO)
        loaded["objects"].append(dict(loaded["objects"][0]))
        summary = validate_cell_definition(loaded, DEMO, parser, notes)
        self.assertFalse(summary.ok)
        self.assertTrue(any("Duplicate id" in e for e in summary.errors))

    def test_missing_destination_fails(self):
        loaded, parser, notes = load_yaml(DEMO)
        loaded["task"]["destinations"] = []
        summary = validate_cell_definition(loaded, DEMO, parser, notes)
        self.assertFalse(summary.ok)
        self.assertTrue(any("task.destinations" in e for e in summary.errors))

    def test_discovery_finds_cell_definition_template(self):
        payload = discover_all()
        self.assertTrue(any("demo_ur5_sorting_cell" in c["path"] for c in payload["cell_definitions"]))

    def test_no_duplicate_assets_or_scenes_dirs_created(self):
        self.assertTrue((REPO_ROOT / "assets").is_dir())
        self.assertTrue((REPO_ROOT / "scenes").is_dir())
        self.assertFalse((REPO_ROOT / "assets_2").exists())
        self.assertFalse((REPO_ROOT / "scenes_2").exists())


if __name__ == "__main__":
    unittest.main()
