#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "detected_objects"
TASK_FIXTURES = REPO_ROOT / "tests" / "fixtures" / "task_recipes"
PIPELINE = REPO_ROOT / "scripts" / "run_perception_task_pipeline.py"


class PerceptionTaskPipelineTests(unittest.TestCase):
    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(PIPELINE), *args], capture_output=True, text=True, check=False)

    def test_pipeline_produces_runtime_plan_and_bridge_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out_dir = Path(tmpdir) / "reports"
            proc = self._run(
                "--task-recipe",
                str(TASK_FIXTURES / "valid_sort_by_colour.yaml"),
                "--detected-objects",
                str(FIXTURES / "valid_epd_colour_sorting.yaml"),
                "--output-dir",
                str(out_dir),
                "--dry-run",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            payload = json.loads(proc.stdout)
            self.assertIn(payload["status"], {"PASS", "WARN"})
            self.assertTrue((out_dir / "runtime_execution_plan.json").is_file())
            self.assertTrue((out_dir / "emd_grasp_bridge_payload.json").is_file())
            self.assertTrue((out_dir / "detected_objects_validation_report.json").is_file())

    def test_pipeline_fails_for_missing_pose(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            proc = self._run(
                "--task-recipe",
                str(TASK_FIXTURES / "valid_pick_place.yaml"),
                "--detected-objects",
                str(FIXTURES / "missing_pose_fail.yaml"),
                "--output-dir",
                str(Path(tmpdir) / "reports"),
            )
            self.assertEqual(proc.returncode, 1)
            payload = json.loads(proc.stdout)
            self.assertEqual(payload["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
