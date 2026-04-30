#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "replay_emd_bridge_payload.py"


class ReplayEmdBridgePayloadTests(unittest.TestCase):
    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True, check=False)

    def _payload(self, **overrides: object) -> dict[str, object]:
        base: dict[str, object] = {
            "schema_version": "emd_grasp_bridge_payload/v1",
            "scene_package": "ur5_2f_test",
            "grasp_task": {
                "task_id": "demo",
                "grasp_targets": [
                    {
                        "object_id": "obj_1",
                        "target_type": "box",
                        "target_pose": {"frame_id": "world", "xyz": [0, 0, 0], "rpy": [0, 0, 0]},
                        "target_shape": {"type": "BOX", "dimensions": [0.1, 0.1, 0.1]},
                        "grasp_methods": [{"ee_id": "robotiq_2f", "grasp_poses": [{"frame_id": "world", "xyz": [0, 0, 0], "rpy": [0, 0, 0]}], "grasp_ranks": [1.0]}],
                        "destination_id": "red_bin",
                        "destination_pose": {"frame_id": "world", "xyz": [0.2, 0.0, 0.1], "rpy": [0, 0, 0]},
                    }
                ],
            },
        }
        base.update(overrides)
        return base

    def test_valid_payload_dry_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "payload.json"
            p.write_text(json.dumps(self._payload()), encoding="utf-8")
            proc = self._run("--payload", str(p), "--scene-package", "ur5_2f_test", "--dry-run")
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertIn("PASS: dry-run only", proc.stdout)
            self.assertIn("explicit destination release pose will be used", proc.stdout)

    def test_missing_payload_file(self) -> None:
        proc = self._run("--payload", "/tmp/does-not-exist.json", "--scene-package", "ur5_2f_test", "--dry-run")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("Payload file not found", proc.stdout)

    def test_missing_destination_pose_warning(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            payload = self._payload()
            target = payload["grasp_task"]["grasp_targets"][0]
            target["destination_pose"] = {"frame_id": "world"}
            p = Path(tmp) / "payload.json"
            p.write_text(json.dumps(payload), encoding="utf-8")
            proc = self._run("--payload", str(p), "--scene-package", "ur5_2f_test", "--dry-run")
            self.assertEqual(proc.returncode, 0)
            self.assertIn("legacy release fallback will be used", proc.stdout)

    def test_invalid_destination_pose_xyz_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            payload = self._payload()
            target = payload["grasp_task"]["grasp_targets"][0]
            target["destination_pose"] = {"frame_id": "world", "xyz": [0.2, 0.0]}
            p = Path(tmp) / "payload.json"
            p.write_text(json.dumps(payload), encoding="utf-8")
            proc = self._run("--payload", str(p), "--scene-package", "ur5_2f_test", "--dry-run")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("destination pose xyz must be a 3-number list", proc.stdout)

    def test_missing_frame_id_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            payload = self._payload()
            target = payload["grasp_task"]["grasp_targets"][0]
            target["grasp_methods"][0]["grasp_poses"][0].pop("frame_id", None)
            p = Path(tmp) / "payload.json"
            p.write_text(json.dumps(payload), encoding="utf-8")
            proc = self._run("--payload", str(p), "--scene-package", "ur5_2f_test", "--dry-run")
            self.assertEqual(proc.returncode, 1)
            self.assertIn("grasp pose frame_id is missing", proc.stdout)

    def test_scene_package_mismatch_warning(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "payload.json"
            p.write_text(json.dumps(self._payload(scene_package="other_scene")), encoding="utf-8")
            proc = self._run("--payload", str(p), "--scene-package", "ur5_2f_test", "--dry-run")
            self.assertEqual(proc.returncode, 0)
            self.assertIn("scene_package mismatch", proc.stdout)


if __name__ == "__main__":
    unittest.main()
