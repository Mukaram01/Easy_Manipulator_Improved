#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "run_generated_cell_acceptance.py"
TASK = REPO_ROOT / "tests" / "fixtures" / "task_recipes" / "mvp1_generated_cell_colour_sorting.yaml"
TASK_MISSING_POSE = REPO_ROOT / "tests" / "fixtures" / "task_recipes" / "mvp1_generated_cell_colour_sorting_missing_reject_pose.yaml"
OBJECTS = REPO_ROOT / "tests" / "fixtures" / "detected_objects" / "mvp1_colour_sorting_with_fallback.yaml"


class GeneratedCellAcceptanceTests(unittest.TestCase):
    def _run(self, strict: bool = False, task: Path = TASK) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as tmpdir:
            cmd = [
                sys.executable,
                str(SCRIPT),
                "--scene-package",
                "ur5_robotiq_generated_cell",
                "--task-recipe",
                str(task),
                "--detected-objects",
                str(OBJECTS),
                "--output-dir",
                tmpdir,
                "--json",
            ]
            if strict:
                cmd.append("--strict")
            return subprocess.run(cmd, capture_output=True, text=True, check=False)

    def test_valid_colour_routing_example(self) -> None:
        proc = self._run(task=TASK)
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertEqual(payload["selected_object"], "red_001")
        self.assertEqual(payload["matched_rule"], "route_red")
        self.assertEqual(payload["destination_selected"], "red_bin")

    def test_fallback_destination_is_present_in_plan(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            proc = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    "--scene-package",
                    "ur5_robotiq_generated_cell",
                    "--task-recipe",
                    str(TASK),
                    "--detected-objects",
                    str(OBJECTS),
                    "--output-dir",
                    tmpdir,
                    "--json",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            plan = json.loads((Path(tmpdir) / "runtime_execution_plan.json").read_text(encoding="utf-8"))
            destinations = [step["routing"]["destination_id"] for step in plan["steps"]]
            self.assertIn("reject_bin", destinations)

    def test_missing_destination_pose_warning(self) -> None:
        proc = self._run(task=TASK_MISSING_POSE)
        payload = json.loads(proc.stdout)
        self.assertTrue(any("pose_xyz is missing" in w for w in payload["warnings"]))

    def test_missing_generated_scene_warning(self) -> None:
        proc = self._run()
        payload = json.loads(proc.stdout)
        self.assertTrue(any("not found" in w for w in payload["warnings"]))

    def test_strict_mode_fails_on_warnings(self) -> None:
        proc = self._run(strict=True, task=TASK_MISSING_POSE)
        self.assertEqual(proc.returncode, 1)
        payload = json.loads(proc.stdout)
        self.assertEqual(payload["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
