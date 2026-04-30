#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from scripts.workcell_discovery import discover_all, discover_detected_objects, discover_task_recipes


class WorkcellDiscoveryTests(unittest.TestCase):
    def test_finds_task_recipe_fixtures(self):
        records = discover_task_recipes()
        self.assertTrue(any("mvp1_generated_cell_colour_sorting" in r.path for r in records))

    def test_finds_detected_objects_fixtures(self):
        records = discover_detected_objects()
        self.assertTrue(any("mvp1_colour_sorting_with_fallback" in r.path for r in records))

    def test_json_compatible_records(self):
        payload = discover_all()
        json.dumps(payload)
        self.assertIn("scenes", payload)

    def test_summary_mode_does_not_crash(self):
        script = Path(__file__).resolve().parents[1] / "scripts/workcell_discovery.py"
        p = subprocess.run([sys.executable, str(script), "--summary"], capture_output=True, text=True, check=False)
        self.assertEqual(p.returncode, 0)

    def test_ur5_scene_not_flagged_missing_config(self):
        scenes = discover_all()["scenes"]
        ur5 = next((s for s in scenes if s["package_name"] == "ur5_2f_test"), None)
        self.assertIsNotNone(ur5)
        self.assertFalse(any("missing config" in w for w in ur5.get("warnings", [])))

    def test_fixture_categories_separate_valid_from_failure_tests(self):
        recipes = discover_task_recipes()
        valid = next(r for r in recipes if r.display_name == "valid_garbage_sorting" and "tests/fixtures/task_recipes" in r.path)
        fail_case = next(r for r in recipes if r.display_name == "fail_missing_destination")
        self.assertEqual(valid.category, "valid")
        self.assertEqual(fail_case.category, "failure_test")

    def test_invalid_yaml_becomes_warning(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "bad.yaml"
            p.write_text("::::", encoding="utf-8")
            from scripts import workcell_discovery as d

            original = d.REPO_ROOT
            d.REPO_ROOT = Path(td)
            try:
                recs = d.discover_task_recipes()
                self.assertEqual(recs, [])
            finally:
                d.REPO_ROOT = original


if __name__ == "__main__":
    unittest.main()
