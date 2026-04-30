#!/usr/bin/env python3
from __future__ import annotations

import json
import unittest
from unittest.mock import patch

from scripts.run_cell_readiness_check import main
from scripts.run_cell_cycle_panel import build_preflight_command


class ReadinessTests(unittest.TestCase):
    def test_offline_valid(self):
        rc = main([
            "--scene-package", "ur5_2f_test",
            "--task-recipe", "tests/fixtures/task_recipes/valid_garbage_sorting.yaml",
            "--detected-objects", "tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml",
            "--json",
        ])
        self.assertIn(rc, (0,))

    def test_missing_task_recipe_fails(self):
        rc = main(["--scene-package", "ur5_2f_test", "--task-recipe", "missing.yaml", "--json"])
        self.assertEqual(rc, 1)

    @patch("scripts.run_cell_readiness_check._check_topic_exists", return_value=False)
    def test_live_missing_topics_fails(self, _mock):
        rc = main(["--scene-package", "ur5_2f_test", "--task-recipe", "tests/fixtures/task_recipes/valid_garbage_sorting.yaml", "--live", "--json"])
        self.assertEqual(rc, 1)

    @patch("scripts.run_cell_readiness_check._check_topic_exists", return_value=True)
    @patch("scripts.run_cell_readiness_check._check_tf_available", return_value=False)
    def test_live_missing_tf_fails_when_enabled(self, _tf, _topic):
        rc = main(["--scene-package", "ur5_2f_test", "--task-recipe", "tests/fixtures/task_recipes/valid_garbage_sorting.yaml", "--live", "--check-tf", "--json"])
        self.assertEqual(rc, 1)

    def test_preflight_command_generation(self):
        cmd = build_preflight_command({
            "scene_package": "ur5_2f_test",
            "task_recipe": "tests/fixtures/task_recipes/valid_garbage_sorting.yaml",
            "detected_objects": "tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml",
            "capture_live": True,
            "epd_topic": "/easy_perception_deployment/epd_localize_output",
        })
        joined = " ".join(cmd)
        self.assertIn("run_cell_readiness_check.py", joined)
        self.assertIn("--scene-package ur5_2f_test", joined)
        self.assertIn("--live", joined)


if __name__ == "__main__":
    unittest.main()
