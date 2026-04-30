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
