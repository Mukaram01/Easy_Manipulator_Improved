#!/usr/bin/env python3
"""Tests for offline capability contract tooling."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "capabilities"
CATALOG_DIR = REPO_ROOT / "catalog" / "capabilities"
CATALOG_SUBDIRECTORIES = ["robots", "end_effectors", "sensors", "tasks", "environment_assets"]


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


validator = _load_module(
    "validate_capability_contracts", REPO_ROOT / "scripts" / "validate_capability_contracts.py"
)


class CapabilityFixturesTests(unittest.TestCase):
    def _assert_pass(self, target: Path) -> None:
        results = validator.validate_path(target)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].status, "PASS", f"{target.name}: {results[0].errors} {results[0].warnings}")

    def _ids_for_schema_prefix(self, directory: Path, schema_prefix: str, key: str) -> set[str]:
        ids: set[str] = set()
        for result in validator.validate_path(directory):
            self.assertEqual(result.status, "PASS", f"{result.path}: {result.errors} {result.warnings}")
            contract, _notes = validator.load_contract(Path(result.path))
            schema_version = str(contract.get("schema_version", ""))
            if schema_version.startswith(schema_prefix) and isinstance(contract.get(key), dict):
                item_id = contract[key].get("id")
                if isinstance(item_id, str):
                    ids.add(item_id)
        return ids

    def test_catalog_directory_exists(self) -> None:
        self.assertTrue(CATALOG_DIR.exists())
        self.assertTrue(CATALOG_DIR.is_dir())

    def test_catalog_subdirectories_exist(self) -> None:
        for subdir in CATALOG_SUBDIRECTORIES:
            path = CATALOG_DIR / subdir
            self.assertTrue(path.exists(), f"Missing {subdir}")
            self.assertTrue(path.is_dir(), f"Not a directory: {subdir}")

    def test_catalog_directory_validation(self) -> None:
        results = validator.validate_path(CATALOG_DIR)
        self.assertGreaterEqual(len(results), 10)
        self.assertTrue(all(r.status == "PASS" for r in results))

    def test_catalog_has_all_capability_kinds(self) -> None:
        self.assertGreaterEqual(len(self._ids_for_schema_prefix(CATALOG_DIR, "robot_capability/", "robot")), 1)
        self.assertGreaterEqual(len(self._ids_for_schema_prefix(CATALOG_DIR, "end_effector_capability/", "end_effector")), 1)
        self.assertGreaterEqual(len(self._ids_for_schema_prefix(CATALOG_DIR, "sensor_capability/", "sensor")), 1)
        self.assertGreaterEqual(len(self._ids_for_schema_prefix(CATALOG_DIR, "task_capability/", "task")), 1)
        self.assertGreaterEqual(len(self._ids_for_schema_prefix(CATALOG_DIR, "environment_asset/", "asset")), 1)

    def test_catalog_contains_important_ids(self) -> None:
        robot_ids = self._ids_for_schema_prefix(CATALOG_DIR, "robot_capability/", "robot")
        end_effector_ids = self._ids_for_schema_prefix(CATALOG_DIR, "end_effector_capability/", "end_effector")
        sensor_ids = self._ids_for_schema_prefix(CATALOG_DIR, "sensor_capability/", "sensor")
        asset_ids = self._ids_for_schema_prefix(CATALOG_DIR, "environment_asset/", "asset")

        self.assertIn("ur5", robot_ids)
        self.assertTrue(any("delta" in robot_id for robot_id in robot_ids))
        self.assertTrue(any("2f" in end_effector_id or "robotiq_2f" in end_effector_id for end_effector_id in end_effector_ids))
        self.assertTrue(any("airpick" in end_effector_id or "suction" in end_effector_id for end_effector_id in end_effector_ids))
        self.assertTrue(any("realsense_d435i" in sensor_id for sensor_id in sensor_ids))
        self.assertTrue(any("conveyor" in asset_id for asset_id in asset_ids))

    def test_valid_robot_fixtures(self) -> None:
        for name in [
            "robot_ur5.yaml",
            "robot_generic_delta.yaml",
            "robot_generic_scara.yaml",
            "robot_generic_gantry.yaml",
        ]:
            self._assert_pass(FIXTURE_DIR / name)

    def test_valid_end_effector_fixtures(self) -> None:
        for name in [
            "ee_robotiq_2f.yaml",
            "ee_robotiq_3f.yaml",
            "ee_airpick_suction.yaml",
            "ee_vacuum_array.yaml",
        ]:
            self._assert_pass(FIXTURE_DIR / name)

    def test_valid_sensor_fixtures(self) -> None:
        for name in ["sensor_realsense_d435i.yaml", "sensor_overhead_rgbd.yaml"]:
            self._assert_pass(FIXTURE_DIR / name)

    def test_valid_task_fixtures(self) -> None:
        for name in [
            "task_pick_place.yaml",
            "task_colour_sorting.yaml",
            "task_shape_sorting.yaml",
            "task_garbage_sorting.yaml",
            "task_conveyor_sorting.yaml",
            "task_magnetic_pick_place.yaml",
        ]:
            self._assert_pass(FIXTURE_DIR / name)

    def test_valid_asset_fixtures(self) -> None:
        for name in ["asset_table.yaml", "asset_conveyor.yaml", "asset_bin.yaml", "asset_machine_fixture.yaml"]:
            self._assert_pass(FIXTURE_DIR / name)

    def test_fixture_directory_validation(self) -> None:
        results = validator.validate_path(FIXTURE_DIR)
        self.assertGreaterEqual(len(results), 10)
        self.assertTrue(all(r.status == "PASS" for r in results))

    def test_invalid_missing_schema_version(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bad.yaml"
            path.write_text("robot:\n  id: bad\n", encoding="utf-8")
            result = validator.validate_path(path)[0]
            self.assertEqual(result.status, "FAIL")
            self.assertTrue(any("schema_version" in err for err in result.errors))

    def test_invalid_required_field(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bad.yaml"
            path.write_text(
                "schema_version: robot_capability/v1\nrobot:\n  id: x\n  label: X\n  brand: X\n",
                encoding="utf-8",
            )
            result = validator.validate_path(path)[0]
            self.assertEqual(result.status, "FAIL")
            self.assertTrue(any("robot.family" in err for err in result.errors))

    def test_unknown_enum_warns_unless_strict(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "custom_enum.yaml"
            path.write_text(
                """schema_version: robot_capability/v1
robot:
  id: custom_bot
  label: Custom Bot
  brand: Internal
  family: snake_arm
  mounting: floor
  planning_groups: [arm]
  base_frame: base
  default_tool_frame: tool0
""",
                encoding="utf-8",
            )
            warn_result = validator.validate_path(path, strict=False)[0]
            strict_result = validator.validate_path(path, strict=True)[0]
            self.assertEqual(warn_result.status, "WARN")
            self.assertEqual(strict_result.status, "FAIL")

    def test_json_output(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "validate_capability_contracts.py"), str(FIXTURE_DIR), "--json"],
            check=True,
            text=True,
            capture_output=True,
        )
        payload = json.loads(completed.stdout)
        self.assertIn("summary", payload)
        self.assertGreaterEqual(payload["summary"]["pass"], 1)

    def test_check_scripts_have_valid_bash_syntax(self) -> None:
        subprocess.run(
            ["bash", "-n", str(REPO_ROOT / "scripts" / "check_capability_contracts.sh"), str(REPO_ROOT / "scripts" / "preflight_workcell.sh")],
            check=True,
            text=True,
            capture_output=True,
        )


if __name__ == "__main__":
    unittest.main()
