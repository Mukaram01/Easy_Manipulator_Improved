#!/usr/bin/env python3
from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.run_cell_cycle_panel import (
    DEFAULTS,
    build_cycle_command,
    build_gated_cycle_command,
    build_generate_workcell_command,
    build_validate_cell_definition_command,
    parse_cycle_report,
)
from scripts.workcell_discovery import discover_all


class RunCellCyclePanelTests(unittest.TestCase):
    def test_operator_defaults_use_valid_fixtures(self):
        self.assertTrue(DEFAULTS["task_recipe"].endswith("valid_garbage_sorting.yaml"))
        self.assertTrue(DEFAULTS["detected_objects"].endswith("valid_epd_garbage_sorting.yaml"))
        self.assertEqual(DEFAULTS["output_dir"], "/tmp/mvp1_live_smoke_test")

    def test_filtered_defaults_avoid_failure_test_fixtures(self):
        payload = discover_all()
        task_values = [x["path"] for x in payload["task_recipes"] if x.get("category") != "failure_test"]
        obj_values = [x["path"] for x in payload["detected_objects"] if x.get("category") != "failure_test"]
        self.assertNotIn("tests/fixtures/task_recipes/fail_missing_destination.yaml", task_values)
        self.assertNotIn("tests/fixtures/detected_objects/low_confidence_object.yaml", obj_values)

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
            "frame_fallback": "world",
            "dry_run": True,
            "replay": False,
            "strict": False,
            "json": True,
        }
        cmd = build_cycle_command(cfg)
        self.assertIn("--dry-run", cmd)
        self.assertNotIn("--replay", cmd)
        self.assertIn("--no-replay", cmd)
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
            "frame_fallback": "world",
            "dry_run": True,
            "replay": False,
            "strict": False,
            "json": True,
        }
        cmd = build_cycle_command(cfg)
        self.assertIn("--capture-live", cmd)
        self.assertIn("--epd-topic", cmd)
        self.assertIn("--dry-run", cmd)
        self.assertIn("--no-replay", cmd)
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
                "perception_source": "live_epd",
                "detected_objects_used": "/tmp/x/detected_objects_used.yaml",
                "acceptance": {"errors": ["e1"]},
            }
            p = Path(td) / "cycle_report.json"
            p.write_text(json.dumps(report), encoding="utf-8")
            summary = parse_cycle_report(Path(td))
            self.assertEqual(summary.status, "WARN")
            self.assertEqual(summary.warning_count, 2)
            self.assertEqual(summary.error_count, 1)
            self.assertEqual(summary.perception_source, "live_epd")

    def test_panel_gated_command_includes_require_preflight(self):
        cfg = {"scene_package":"ur5_2f_test","task_recipe":"t.yaml","detected_objects":"d.yaml","capture_live":False,"epd_topic":"/epd","output_dir":"/tmp/mvp1","capture_timeout":10,"min_objects":1,"frame_fallback":"world","dry_run":True,"replay":False,"strict":False,"json":True}
        cmd = build_gated_cycle_command(cfg)
        self.assertIn('--require-preflight', cmd)
        self.assertNotIn('--preflight-check-ros-topics', cmd)

    def test_panel_live_gated_command_has_live_preflight_flags(self):
        cfg = {"scene_package":"ur5_2f_test","task_recipe":"t.yaml","detected_objects":"d.yaml","capture_live":True,"epd_topic":"/epd","output_dir":"/tmp/mvp1","capture_timeout":10,"min_objects":1,"frame_fallback":"world","dry_run":True,"replay":False,"strict":False,"json":True}
        cmd = build_gated_cycle_command(cfg)
        self.assertIn('--preflight-live', cmd)
        self.assertIn('--preflight-check-tf', cmd)
        self.assertIn('--preflight-check-ros-topics', cmd)

    def test_panel_cell_definition_commands(self):
        vcmd = build_validate_cell_definition_command("cell_definitions/demo_ur5_sorting_cell.yaml")
        gcmd = build_generate_workcell_command("cell_definitions/demo_ur5_sorting_cell.yaml", "/tmp/generated_workcells/demo")
        self.assertIn("validate_cell_definition.py", " ".join(vcmd))
        self.assertIn("--cell-definition", vcmd)
        self.assertIn("generate_workcell_from_cell_definition.py", " ".join(gcmd))


if __name__ == "__main__":
    unittest.main()
