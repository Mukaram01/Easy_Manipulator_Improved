#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures" / "emd_grasp_bridge"
BRIDGE = REPO_ROOT / "scripts" / "convert_runtime_plan_to_emd_grasp.py"


class EmdGraspBridgeTests(unittest.TestCase):
    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(BRIDGE), *args], capture_output=True, text=True, check=False)

    def _run_json(self, fixture: str, *extra: str) -> tuple[subprocess.CompletedProcess[str], dict]:
        proc = self._run("--runtime-plan", str(FIXTURES / fixture), "--json", *extra)
        payload = json.loads(proc.stdout)
        return proc, payload

    def test_valid_conversion_produces_bridge_schema(self) -> None:
        proc, payload = self._run_json("valid_single_box_runtime_plan.json")
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        self.assertEqual(payload["schema_version"], "emd_grasp_bridge_payload/v1")
        self.assertEqual(payload["status"], "PASS")

    def test_one_pick_step_maps_to_one_target(self) -> None:
        _, payload = self._run_json("valid_single_box_runtime_plan.json")
        self.assertEqual(len(payload["grasp_task"]["grasp_targets"]), 1)

    def test_multiple_pick_steps_map_to_multiple_targets(self) -> None:
        _, payload = self._run_json("valid_colour_sort_multi_runtime_plan.json")
        self.assertEqual(len(payload["grasp_task"]["grasp_targets"]), 2)

    def test_rejected_objects_are_skipped_with_warning(self) -> None:
        _, payload = self._run_json("rejected_unknown_skip_runtime_plan.json")
        self.assertEqual(payload["summary"]["skipped_objects"], 1)
        self.assertEqual(len(payload["grasp_task"]["grasp_targets"]), 1)
        self.assertTrue(any("Skipping rejected/unknown object" in w for w in payload.get("warnings", [])))

    def test_missing_dimensions_warns_and_strict_fails(self) -> None:
        proc, payload = self._run_json("missing_dimensions_warn_runtime_plan.json")
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        self.assertEqual(payload["status"], "WARN")
        strict_proc, strict_payload = self._run_json("missing_dimensions_warn_runtime_plan.json", "--strict")
        self.assertEqual(strict_proc.returncode, 1)
        self.assertEqual(strict_payload["status"], "FAIL")

    def test_missing_pose_fails(self) -> None:
        proc, payload = self._run_json("missing_pose_fail_runtime_plan.json")
        self.assertEqual(proc.returncode, 1)
        self.assertEqual(payload["status"], "FAIL")
        self.assertTrue(any("missing a complete pose" in e for e in payload.get("errors", [])))

    def test_synthesized_grasp_pose_warns(self) -> None:
        _, payload = self._run_json("synthesized_grasp_pose_warn_runtime_plan.json")
        self.assertEqual(payload["status"], "WARN")
        self.assertTrue(any("synthesized conservative grasp pose" in w for w in payload.get("warnings", [])))

    def test_destination_metadata_is_preserved(self) -> None:
        _, payload = self._run_json("valid_single_box_runtime_plan.json")
        target = payload["grasp_task"]["grasp_targets"][0]
        self.assertEqual(target["destination_id"], "red_bin")
        self.assertEqual(target["destination_pose"]["frame_id"], "world")
        self.assertTrue(any("destination pose is preserved" in n for n in target.get("notes", [])))

    def test_json_output_is_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "payload.json"
            first = self._run("--runtime-plan", str(FIXTURES / "valid_single_box_runtime_plan.json"), "--output", str(out), "--json")
            second = self._run("--runtime-plan", str(FIXTURES / "valid_single_box_runtime_plan.json"), "--output", str(out), "--json")
            self.assertEqual(first.returncode, 0)
            self.assertEqual(second.returncode, 0)
            payload_a = json.loads(first.stdout)
            payload_b = json.loads(second.stdout)
            self.assertEqual(payload_a["grasp_task"], payload_b["grasp_task"])
            self.assertEqual(payload_a["ros_interface"], payload_b["ros_interface"])

    def test_shape_mapping_box_cylinder_and_sphere_fallback(self) -> None:
        _, payload = self._run_json("shape_mapping_cases_runtime_plan.json")
        by_id = {item["object_id"]: item for item in payload["grasp_task"]["grasp_targets"]}
        self.assertEqual(by_id["obj_box"]["target_shape"]["type"], "BOX")
        self.assertEqual(by_id["obj_cyl"]["target_shape"]["type"], "CYLINDER")
        self.assertEqual(by_id["obj_sphere_ambiguous"]["target_shape"]["type"], "BOX")
        self.assertTrue(any("sphere shape" in w for w in payload.get("warnings", [])))

    def test_ros_mode_is_guarded_and_lazy(self) -> None:
        proc, payload = self._run_json("valid_single_box_runtime_plan.json", "--mode", "ros")
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        self.assertEqual(payload["mode"], "ros")
        self.assertTrue(any("dry-run enabled" in w for w in payload.get("warnings", [])))


if __name__ == "__main__":
    unittest.main()
