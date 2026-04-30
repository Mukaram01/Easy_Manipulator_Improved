#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.run_cell_cycle_panel import build_cycle_command, parse_cycle_report


class RunCellCyclePanelTests(unittest.TestCase):
    def test_defaults_encode_dry_run_enabled_replay_disabled(self):
        cfg = {
            "scene_package": "ur5_2f_test",
            "task_recipe": "tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml",
            "detected_objects": "tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml",
            "capture_live": False,
            "epd_topic": "/easy_perception_deployment/epd_localize_output",
            "output_dir": "/tmp/mvp1",
            "capture_timeout": 10,
            "min_objects": 1,
            "dry_run": True,
            "replay": False,
            "strict": False,
            "json": True,
        }
        cmd = build_cycle_command(cfg)
        self.assertIn("--dry-run", cmd)
        self.assertNotIn("--replay", cmd)
        self.assertIn("--detected-objects", cmd)

    def test_live_mode_command_uses_capture_live(self):
        cfg = {
            "scene_package": "ur5_2f_test",
            "task_recipe": "t.yaml",
            "detected_objects": "d.yaml",
            "capture_live": True,
            "epd_topic": "/epd",
            "output_dir": "/tmp/mvp1",
            "capture_timeout": 10,
            "min_objects": 1,
            "dry_run": True,
            "replay": False,
            "strict": False,
            "json": True,
        }
        cmd = build_cycle_command(cfg)
        self.assertIn("--capture-live", cmd)
        self.assertIn("--epd-topic", cmd)
        self.assertNotIn("--detected-objects", cmd)

    def test_missing_cycle_report_is_graceful(self):
        with tempfile.TemporaryDirectory() as td:
            summary = parse_cycle_report(Path(td))
            self.assertEqual(summary.status, "FAIL")
            self.assertIsNone(summary.report_path)

    def test_status_warning_error_parsing(self):
        with tempfile.TemporaryDirectory() as td:
            report = {
                "status": "WARN",
                "warnings": ["w1", "w2"],
                "acceptance": {"errors": ["e1"]},
            }
            p = Path(td) / "cycle_report.json"
            p.write_text(json.dumps(report), encoding="utf-8")
            summary = parse_cycle_report(Path(td))
            self.assertEqual(summary.status, "WARN")
            self.assertEqual(summary.warning_count, 2)
            self.assertEqual(summary.error_count, 1)


if __name__ == "__main__":
    unittest.main()
