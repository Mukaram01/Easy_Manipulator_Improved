#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "detected_objects"
VALIDATOR = REPO_ROOT / "scripts" / "validate_detected_objects.py"
ADAPTER = REPO_ROOT / "scripts" / "run_task_recipe_adapter.py"
TASK_FIXTURES = REPO_ROOT / "tests" / "fixtures" / "task_recipes"
CAPTURE = REPO_ROOT / "scripts" / "capture_epd_detected_objects.py"


class DetectedObjectsToolsTests(unittest.TestCase):
    def _run(self, script: Path, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(script), *args], capture_output=True, text=True, check=False)

    def test_validator_pass_warn_fail_and_strict(self) -> None:
        good = self._run(VALIDATOR, str(FIXTURES / "valid_epd_single_box.yaml"), "--json")
        self.assertEqual(good.returncode, 0, msg=good.stdout + good.stderr)
        self.assertEqual(json.loads(good.stdout)["status"], "PASS")

        warn = self._run(VALIDATOR, str(FIXTURES / "missing_dimensions_warn.yaml"), "--json")
        self.assertEqual(warn.returncode, 0, msg=warn.stdout + warn.stderr)
        self.assertEqual(json.loads(warn.stdout)["status"], "WARN")

        strict_fail = self._run(VALIDATOR, str(FIXTURES / "missing_dimensions_warn.yaml"), "--json", "--strict")
        self.assertEqual(strict_fail.returncode, 1)
        self.assertEqual(json.loads(strict_fail.stdout)["status"], "FAIL")

        fail = self._run(VALIDATOR, str(FIXTURES / "missing_pose_fail.yaml"), "--json")
        self.assertEqual(fail.returncode, 1)
        self.assertEqual(json.loads(fail.stdout)["status"], "FAIL")

    def test_adapter_accepts_detected_objects_and_routes(self) -> None:
        proc = self._run(
            ADAPTER,
            "--task-recipe",
            str(TASK_FIXTURES / "valid_sort_by_colour.yaml"),
            "--objects",
            str(FIXTURES / "valid_epd_colour_sorting.yaml"),
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        destinations = [step["routing"]["destination_id"] for step in payload["steps"]]
        self.assertEqual(destinations, ["red_bin", "blue_bin"])

    def test_low_confidence_routes_to_reject(self) -> None:
        proc = self._run(
            ADAPTER,
            "--task-recipe",
            str(TASK_FIXTURES / "valid_garbage_sorting.yaml"),
            "--objects",
            str(FIXTURES / "low_confidence_object.yaml"),
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertEqual(payload["steps"][0]["routing"]["destination_id"], "unknown_reject_bin")

    def test_capture_script_ros_imports_are_lazy(self) -> None:
        text = CAPTURE.read_text(encoding="utf-8")
        main_idx = text.find("def main")
        import_idx = text.find("import rclpy")
        self.assertGreater(import_idx, main_idx)


if __name__ == "__main__":
    unittest.main()
