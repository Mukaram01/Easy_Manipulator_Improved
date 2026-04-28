#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TASK_FIXTURES = REPO_ROOT / "tests" / "fixtures" / "task_recipes"
OBJECT_FIXTURES = REPO_ROOT / "tests" / "fixtures" / "runtime_objects"
ADAPTER = REPO_ROOT / "scripts" / "run_task_recipe_adapter.py"


class TaskRecipeAdapterTests(unittest.TestCase):
    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(ADAPTER), *args],
            capture_output=True,
            text=True,
            check=False,
        )

    def test_sort_by_colour_routes_expected_bins(self) -> None:
        proc = self._run(
            "--task-recipe",
            str(TASK_FIXTURES / "valid_sort_by_colour.yaml"),
            "--objects",
            str(OBJECT_FIXTURES / "sort_by_colour.yaml"),
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertEqual(payload["schema_version"], "runtime_execution_plan/v1")
        destinations = [step["routing"]["destination_id"] for step in payload["steps"]]
        self.assertEqual(destinations, ["red_bin", "blue_bin", "green_bin"])

    def test_garbage_low_confidence_routes_to_reject(self) -> None:
        proc = self._run(
            "--task-recipe",
            str(TASK_FIXTURES / "valid_garbage_sorting.yaml"),
            "--objects",
            str(OBJECT_FIXTURES / "garbage_sorting.yaml"),
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        reject = [step for step in payload["steps"] if step["object"]["id"] == "reject_001"][0]
        self.assertEqual(reject["routing"]["destination_id"], "unknown_reject_bin")
        self.assertIn("pose", reject["routing"]["destination_resolved"])

    def test_sort_by_shape_routes_cylinder_to_cylinder_bin(self) -> None:
        proc = self._run(
            "--task-recipe",
            str(TASK_FIXTURES / "valid_sort_by_shape.yaml"),
            "--objects",
            str(OBJECT_FIXTURES / "sort_by_shape.yaml"),
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        cyl = [step for step in payload["steps"] if step["object"]["id"] == "cyl_001"][0]
        self.assertEqual(cyl["routing"]["destination_id"], "cylinder_bin")
        resolved = cyl["routing"]["destination_resolved"]
        self.assertEqual(resolved["frame_id"], "world")
        self.assertEqual(len(resolved["pose"]["quaternion_xyzw"]), 4)

    def test_destination_resolved_defaults_frame_and_warns_when_pose_missing(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            recipe = Path(tmpdir) / "recipe.json"
            recipe.write_text(
                json.dumps(
                    {
                        "schema_version": "task_recipe/v1",
                        "task": {
                            "id": "fallback_pose_demo",
                            "type": "pick_place",
                            "source": "detected_object",
                            "destinations": [{"id": "place_bin", "label": "place"}],
                            "decision_rules": [
                                {"id": "fallback", "when": {"default": True}, "destination": "place_bin"}
                            ],
                        },
                    }
                ),
                encoding="utf-8",
            )
            proc = self._run(
                "--task-recipe",
                str(recipe),
                "--objects",
                str(OBJECT_FIXTURES / "pick_place_single.yaml"),
                "--json",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            payload = json.loads(proc.stdout)
            self.assertTrue(any("pose_xyz" in w or "no valid pose_xyz" in w for w in payload.get("warnings", [])))
            resolved = payload["steps"][0]["routing"]["destination_resolved"]
            self.assertEqual(resolved["frame_id"], "world")
            self.assertIsNone(resolved["pose"])

    def test_project_dir_auto_discovers_task_recipe(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            project = Path(tmpdir)
            preview_rel = "generated_workcell/demo/generated/task_recipe.preview.yaml"
            preview = project / preview_rel
            preview.parent.mkdir(parents=True, exist_ok=True)
            preview.write_text((TASK_FIXTURES / "valid_pick_place.yaml").read_text(encoding="utf-8"), encoding="utf-8")
            manifest = {
                "schema_version": "workcell_project/v1",
                "artifacts": {"task_preview": preview_rel},
            }
            (project / "project_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

            proc = self._run(
                "--project-dir",
                str(project),
                "--objects",
                str(OBJECT_FIXTURES / "pick_place_single.yaml"),
                "--json",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            payload = json.loads(proc.stdout)
            self.assertEqual(payload["task_recipe"]["id"], "pick_place_demo")
            self.assertEqual(payload["summary"]["routed_count"], 1)

    def test_ros_mode_is_guarded_placeholder(self) -> None:
        proc = self._run(
            "--task-recipe",
            str(TASK_FIXTURES / "valid_pick_place.yaml"),
            "--objects",
            str(OBJECT_FIXTURES / "pick_place_single.yaml"),
            "--mode",
            "ros",
            "--json",
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertTrue(any("guarded placeholder" in warning for warning in payload.get("warnings", [])))


if __name__ == "__main__":
    unittest.main()
