#!/usr/bin/env python3
"""Tests for Cell Definition v1 validation and preview/tooling workflows."""

from __future__ import annotations

import importlib.util
import json
import sys
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


validator = _load_module("validate_cell_definition", REPO_ROOT / "scripts" / "validate_cell_definition.py")
generator = _load_module(
    "generate_scene_from_cell_definition", REPO_ROOT / "scripts" / "generate_scene_from_cell_definition.py"
)
scene_contract_validator = _load_module("validate_scene_contract", REPO_ROOT / "scripts" / "validate_scene_contract.py")
workcell_generator = _load_module(
    "generate_workcell_from_cell_definition", REPO_ROOT / "scripts" / "generate_workcell_from_cell_definition.py"
)


class CellDefinitionValidationTests(unittest.TestCase):
    def _validate_fixture(self, fixture_name: str):
        fixture = FIXTURES / fixture_name
        loaded, parser, notes = validator.load_yaml(fixture)
        return validator.validate_cell_definition(loaded, fixture, parser, notes)

    def test_valid_pick_place_cell_definition_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_pick_place.yaml")
        self.assertTrue(summary.ok)

    def test_valid_sort_by_colour_cell_definition_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_sort_by_colour.yaml")
        self.assertTrue(summary.ok)
    def test_valid_sort_by_colour_with_grasp_strategy_ref_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_sort_by_colour_with_grasp_strategy.yaml")
        self.assertTrue(summary.ok)
    def test_valid_ur5_suction_sorting_fixture_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_ur5_suction_sorting.yaml")
        self.assertTrue(summary.ok)
    def test_valid_generic_cartesian_suction_sorting_fixture_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_generic_cartesian_suction_sorting.yaml")
        self.assertTrue(summary.ok)
    def test_valid_generic_delta_suction_sorting_fixture_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_generic_delta_suction_sorting.yaml")
        self.assertTrue(summary.ok)

    def test_valid_sort_by_shape_cell_definition_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_sort_by_shape.yaml")
        self.assertTrue(summary.ok)

    def test_valid_garbage_sorting_cell_definition_passes(self) -> None:
        summary = self._validate_fixture("cell_definition_garbage_sorting.yaml")
        self.assertTrue(summary.ok)

    def test_missing_safe_joint_state_fails(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_pick_place.yaml")
        del loaded["robot"]["safe_joint_state"]
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_pick_place.yaml", parser, notes)
        self.assertFalse(summary.ok)
        self.assertIn("safe_joint_state", " ".join(summary.errors))

    def test_empty_safe_joint_state_passes_when_home_named_target_exists(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_pick_place.yaml")
        loaded["robot"]["safe_joint_state"] = []
        loaded["robot"]["home_named_target"] = "home"
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_pick_place.yaml", parser, notes)
        self.assertTrue(summary.ok)

    def test_invalid_pose_length_fails(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_pick_place.yaml")
        loaded["objects"][0]["pose_xyz"] = [0.0, 0.0]
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_pick_place.yaml", parser, notes)
        self.assertFalse(summary.ok)
        self.assertIn("pose_xyz", " ".join(summary.errors))

    def test_destination_reference_to_missing_destination_fails(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_sort_by_colour.yaml")
        loaded["task"]["rules"][0]["destination"] = "ghost_bin"
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_sort_by_colour.yaml", parser, notes)
        self.assertFalse(summary.ok)
        self.assertIn("ghost_bin", " ".join(summary.errors))


    def test_missing_robot_model_reports_validation_error_without_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            fixture = Path(tmpdir) / "missing_robot_model.yaml"
            fixture.write_text("schema_version: cell_definition/v1\ncell: {}\n", encoding="utf-8")
            proc = subprocess.run(
                [sys.executable, str(REPO_ROOT / "scripts" / "validate_cell_definition.py"), str(fixture), "--json"],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 1)
            payload = json.loads(proc.stdout)
            self.assertEqual(payload["result"], "FAIL")
            self.assertTrue(any("robot.model must be a non-empty string." in e for e in payload["errors"]))
            self.assertNotIn("Traceback", proc.stderr + proc.stdout)

    def test_unknown_grasp_strategy_warns_by_default(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml")
        loaded["grasp"]["strategy_ref"] = "missing_grasp"
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml", parser, notes, strict=False)
        self.assertTrue(summary.ok)
        self.assertTrue(any("Unknown grasp strategy_ref" in w for w in summary.warnings))

    def test_unknown_grasp_strategy_fails_in_strict_mode(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml")
        loaded["grasp"]["strategy_ref"] = "missing_grasp"
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml", parser, notes, strict=True)
        self.assertFalse(summary.ok)
        self.assertTrue(any("Unknown grasp strategy_ref" in e for e in summary.errors))


class CellDefinitionGenerationTests(unittest.TestCase):
    def test_generated_preview_files_exist_and_scene_contract_check_is_practical(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_sort_by_colour.yaml")
        summary = validator.validate_cell_definition(
            loaded, FIXTURES / "cell_definition_sort_by_colour.yaml", parser, notes
        )
        self.assertTrue(summary.ok)

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir)
            scene_manifest = generator.build_scene_manifest(loaded)
            task_recipe = generator.build_task_recipe(loaded)
            (output_dir / "scene_manifest.preview.yaml").write_text(
                generator._to_yaml_text(scene_manifest), encoding="utf-8"
            )
            (output_dir / "task_recipe.preview.yaml").write_text(
                generator._to_yaml_text(task_recipe), encoding="utf-8"
            )
            (output_dir / "commissioning_summary.md").write_text(
                generator.build_commissioning_summary(loaded, summary.warnings), encoding="utf-8"
            )

            self.assertTrue((output_dir / "scene_manifest.preview.yaml").is_file())
            self.assertTrue((output_dir / "task_recipe.preview.yaml").is_file())
            self.assertTrue((output_dir / "commissioning_summary.md").is_file())

            parsed_scene, parser_name, parser_notes = scene_contract_validator._read_manifest(
                str(output_dir / "scene_manifest.preview.yaml")
            )
            status, notes = scene_contract_validator.validate_task_recipe_block(parsed_scene)
            self.assertIn(status, {"PASS", "WARN"})
            self.assertTrue(parser_name in {"pyyaml", "fallback"})
            self.assertTrue(isinstance(parser_notes + notes, list))

    def test_grasp_strategy_metadata_propagates_to_preview_artifacts(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml")
        summary = validator.validate_cell_definition(
            loaded, FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml", parser, notes
        )
        self.assertTrue(summary.ok)
        scene_manifest = generator.build_scene_manifest(loaded)
        self.assertIn("grasp_strategy", scene_manifest)
        self.assertTrue(scene_manifest["grasp_strategy"]["metadata_only"])
        self.assertFalse(scene_manifest["grasp_strategy"]["runtime_applied"])
        task_recipe = generator.build_task_recipe(loaded)
        self.assertIn("grasp_strategy", task_recipe["pick"])
        commissioning = generator.build_commissioning_summary(loaded, summary.warnings)
        self.assertIn("## Grasp strategy", commissioning)
    def test_suction_metadata_propagates_to_preview_artifacts(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_ur5_suction_sorting.yaml")
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_ur5_suction_sorting.yaml", parser, notes)
        self.assertTrue(summary.ok)
        scene_manifest = generator.build_scene_manifest(loaded)
        ee = scene_manifest["end_effector"]
        self.assertEqual(ee["type"], "suction")
        self.assertTrue(ee["metadata_only"])
        self.assertFalse(ee["runtime_io_applied"])
        task_recipe = generator.build_task_recipe(loaded)
        self.assertEqual(task_recipe["pick"]["allowed_grasp_methods"], ["suction"])
        self.assertIn("grasp_strategy", task_recipe["pick"])
    def test_placeholder_robot_preview_runtime_blockers_present(self) -> None:
        loaded, parser, notes = validator.load_yaml(FIXTURES / "cell_definition_generic_delta_suction_sorting.yaml")
        summary = validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_generic_delta_suction_sorting.yaml", parser, notes)
        self.assertTrue(summary.ok)
        scene_manifest = generator.build_scene_manifest(loaded)
        self.assertEqual(scene_manifest["robot"]["family"], "delta")
        self.assertTrue(scene_manifest["robot"]["preview_only"])
        self.assertFalse(scene_manifest["robot"]["runtime_supported"])
        self.assertTrue(scene_manifest["robot"]["runtime_blockers"])


class WorkcellPackageGenerationTests(unittest.TestCase):
    def _assert_generate_fixture(self, fixture_name: str, package_name: str) -> Path:
        fixture = FIXTURES / fixture_name
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            rc = workcell_generator.generate_package(
                cell_definition_path=fixture,
                output_dir=tmp_path,
                package_name=package_name,
                force=False,
                dry_run=False,
            )
            self.assertEqual(rc, 0)
            package_dir = tmp_path / package_name
            self.assertTrue((package_dir / "package.xml").is_file())
            self.assertTrue((package_dir / "CMakeLists.txt").is_file())
            self.assertTrue((package_dir / "scene_manifest.yaml").is_file())
            self.assertTrue((package_dir / "README.md").is_file())

            parsed_manifest, _, _ = scene_contract_validator._read_manifest(str(package_dir / "scene_manifest.yaml"))
            self.assertIn("self_test", parsed_manifest)
            self.assertIn("task_recipe", parsed_manifest)
            self.assertIn("home_return", parsed_manifest)
            self.assertIn("safe_joint_state", parsed_manifest["home_return"])
            return package_dir

    def test_pick_place_generates_package(self) -> None:
        self._assert_generate_fixture("cell_definition_pick_place.yaml", "generated_pick_place")

    def test_sort_by_colour_generates_package(self) -> None:
        self._assert_generate_fixture("cell_definition_sort_by_colour.yaml", "generated_sort_colour")

    def test_sort_by_shape_generates_package(self) -> None:
        self._assert_generate_fixture("cell_definition_sort_by_shape.yaml", "generated_sort_shape")

    def test_garbage_sorting_generates_package(self) -> None:
        self._assert_generate_fixture("cell_definition_garbage_sorting.yaml", "generated_garbage_sort")
    def test_placeholder_templates_generate_packages(self) -> None:
        self._assert_generate_fixture("cell_definition_generic_cartesian_suction_sorting.yaml", "generated_cart_suction")
        self._assert_generate_fixture("cell_definition_generic_delta_suction_sorting.yaml", "generated_delta_suction")

    def test_dry_run_does_not_write_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            rc = workcell_generator.generate_package(
                cell_definition_path=FIXTURES / "cell_definition_sort_by_colour.yaml",
                output_dir=tmp_path,
                package_name="generated_dry_run",
                force=False,
                dry_run=True,
            )
            self.assertEqual(rc, 0)
            self.assertFalse((tmp_path / "generated_dry_run").exists())

    def test_existing_output_without_force_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            pkg = tmp_path / "generated_exists"
            pkg.mkdir(parents=True, exist_ok=True)
            rc = workcell_generator.generate_package(
                cell_definition_path=FIXTURES / "cell_definition_sort_by_colour.yaml",
                output_dir=tmp_path,
                package_name="generated_exists",
                force=False,
                dry_run=False,
            )
            self.assertNotEqual(rc, 0)

    def test_force_overwrites_generated_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            pkg = tmp_path / "generated_force"
            pkg.mkdir(parents=True, exist_ok=True)
            (pkg / "README.md").write_text("old", encoding="utf-8")
            rc = workcell_generator.generate_package(
                cell_definition_path=FIXTURES / "cell_definition_sort_by_colour.yaml",
                output_dir=tmp_path,
                package_name="generated_force",
                force=True,
                dry_run=False,
            )
            self.assertEqual(rc, 0)
            self.assertNotEqual((pkg / "README.md").read_text(encoding="utf-8"), "old")

    def test_invalid_cell_definition_fails_clearly(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            invalid_path = tmp_path / "invalid.yaml"
            invalid_path.write_text("schema_version: cell_definition/v1\ncell: {}\n", encoding="utf-8")
            rc = workcell_generator.generate_package(
                cell_definition_path=invalid_path,
                output_dir=tmp_path,
                package_name="generated_invalid",
                force=False,
                dry_run=False,
            )
            self.assertNotEqual(rc, 0)

    def test_generated_package_offline_validation_is_practical(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            rc = workcell_generator.generate_package(
                cell_definition_path=FIXTURES / "cell_definition_sort_by_colour.yaml",
                output_dir=tmp_path,
                package_name="generated_validation",
                force=False,
                dry_run=False,
            )
            self.assertEqual(rc, 0)
            manifest_path = tmp_path / "generated_validation" / "scene_manifest.yaml"
            manifest, _, _ = scene_contract_validator._read_manifest(str(manifest_path))
            status, _ = scene_contract_validator.validate_task_recipe_block(manifest)
            self.assertIn(status, {"PASS", "WARN"})

    def test_grasp_strategy_metadata_propagates_to_generated_package_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            rc = workcell_generator.generate_package(
                cell_definition_path=FIXTURES / "cell_definition_sort_by_colour_with_grasp_strategy.yaml",
                output_dir=tmp_path,
                package_name="generated_grasp_strategy",
                force=False,
                dry_run=False,
            )
            self.assertEqual(rc, 0)
            package_dir = tmp_path / "generated_grasp_strategy"
            manifest, _, _ = scene_contract_validator._read_manifest(str(package_dir / "scene_manifest.yaml"))
            self.assertIn("grasp_strategy", manifest)
            task_recipe, _, _ = scene_contract_validator._read_manifest(str(package_dir / "config" / "task_recipe.yaml"))
            self.assertIn("grasp_strategy", task_recipe.get("pick", {}))
            self.assertIn("Grasp strategy", (package_dir / "README.md").read_text(encoding="utf-8"))
            summary_payload = json.loads(
                (package_dir / "generated" / "generated_workcell_summary.json").read_text(encoding="utf-8")
            )
            self.assertIn("grasp_strategy", summary_payload)
            self.assertTrue((package_dir / "generated" / "execution_plan.md").is_file())
            self.assertTrue((package_dir / "generated" / "execution_plan.json").is_file())
            warning_blob = "\n".join(summary_payload.get("warnings", []))
            self.assertNotIn("Execution plan generation status: FAIL", warning_blob)

    def test_unsupported_layout_assets_emit_warning_and_are_not_silently_omitted(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            layout_path = tmp_path / "environment_layout.yaml"
            layout_path.write_text(
                "\n".join(
                    [
                        "schema_version: environment_layout/v1",
                        "layout_id: unsupported_layout",
                        "assets:",
                        "  - id: table_ok",
                        "    type: table",
                        "    pose: {frame: world, xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}",
                        "  - id: lidar_unsupported",
                        "    type: lidar",
                        "    pose: {frame: world, xyz: [1.0, 0.0, 0.5], rpy: [0.0, 0.0, 0.0]}",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            cell_path = tmp_path / "cell.yaml"
            cell_path.write_text(
                (
                    (FIXTURES / "cell_definition_sort_by_colour.yaml").read_text(encoding="utf-8")
                    + f"\nenvironment:\n  frame: world\n  layout: {layout_path}\n  support_surfaces: []\n"
                ),
                encoding="utf-8",
            )
            rc = workcell_generator.generate_package(
                cell_definition_path=cell_path,
                output_dir=tmp_path,
                package_name="generated_with_unsupported_layout",
                force=False,
                dry_run=False,
            )
            self.assertEqual(rc, 0)
            gen = tmp_path / "generated_with_unsupported_layout" / "generated"
            summary = json.loads((gen / "generated_workcell_summary.json").read_text(encoding="utf-8"))
            self.assertTrue(any("Generated preview metadata only: lidar_unsupported" in w for w in summary.get("warnings", [])))
            self.assertTrue(any(a.get("asset_id") == "lidar_unsupported" for a in summary.get("unsupported_assets", [])))
            env_payload, _, _ = scene_contract_validator._read_manifest(str(gen / "generated_environment_objects.yaml"))
            self.assertTrue(any(a.get("asset_id") == "lidar_unsupported" for a in env_payload.get("unsupported_assets", [])))
            self.assertGreater(len(env_payload.get("tracked_assets", [])), 0)


class CellDefinitionWizardTests(unittest.TestCase):
    def setUp(self) -> None:
        self.wizard = REPO_ROOT / "scripts" / "create_cell_definition_wizard.py"
        self.validator_script = REPO_ROOT / "scripts" / "validate_cell_definition.py"

    def _run(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run([sys.executable, str(self.wizard), *args], capture_output=True, text=True, check=False)

    def test_list_templates_succeeds(self) -> None:
        proc = self._run("--list-templates")
        self.assertEqual(proc.returncode, 0)
        self.assertIn("sort_by_colour", proc.stdout)

    def test_list_presets_succeeds(self) -> None:
        proc = self._run("--list-presets")
        self.assertEqual(proc.returncode, 0)
        self.assertIn("robotiq_2f", proc.stdout)

    def test_non_interactive_templates_create_valid_yaml(self) -> None:
        templates = ["pick_place", "sort_by_colour", "sort_by_shape", "garbage_sorting"]
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            for template in templates:
                output = tmp / f"{template}.yaml"
                proc = self._run(
                    "--template", template,
                    "--cell-name", f"Wizard {template}",
                    "--cell-id", f"wizard_{template}",
                    "--robot", "ur5",
                    "--end-effector", "robotiq_2f",
                    "--camera", "realsense_d435i",
                    "--output", str(output),
                    "--force",
                )
                self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
                self.assertTrue(output.is_file())

                valid = subprocess.run([sys.executable, str(self.validator_script), str(output)], capture_output=True, text=True, check=False)
                self.assertEqual(valid.returncode, 0, msg=valid.stdout + valid.stderr)

    def test_dry_run_writes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "dry.yaml"
            proc = self._run(
                "--template", "pick_place",
                "--cell-name", "Dry",
                "--cell-id", "dry",
                "--robot", "ur5",
                "--end-effector", "robotiq_2f",
                "--camera", "realsense_d435i",
                "--output", str(output),
                "--dry-run",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertFalse(output.exists())

    def test_output_without_force_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "out.yaml"
            output.write_text("existing", encoding="utf-8")
            proc = self._run(
                "--template", "pick_place",
                "--cell-name", "No Force",
                "--cell-id", "no_force",
                "--robot", "ur5",
                "--end-effector", "robotiq_2f",
                "--camera", "realsense_d435i",
                "--output", str(output),
            )
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn("Refusing to overwrite", proc.stdout)

    def test_output_with_force_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "out.yaml"
            output.write_text("existing", encoding="utf-8")
            proc = self._run(
                "--template", "pick_place",
                "--cell-name", "Force",
                "--cell-id", "force",
                "--robot", "ur5",
                "--end-effector", "robotiq_2f",
                "--camera", "realsense_d435i",
                "--output", str(output),
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertIn("schema_version: cell_definition/v1", output.read_text(encoding="utf-8"))

    def test_invalid_presets_fail_clearly(self) -> None:
        invalid_cases = [
            ("--template", "unknown_template", "Invalid template"),
            ("--robot", "unknown_robot", "Invalid robot preset"),
            ("--end-effector", "unknown_ee", "Invalid end-effector preset"),
        ]
        for flag, value, expected in invalid_cases:
            with self.subTest(flag=flag):
                args = [
                    "--template", "pick_place",
                    "--cell-name", "Invalid",
                    "--cell-id", "invalid",
                    "--robot", "ur5",
                    "--end-effector", "robotiq_2f",
                    "--camera", "realsense_d435i",
                ]
                idx = args.index(flag) if flag in args else None
                if idx is not None:
                    args[idx + 1] = value
                else:
                    args.extend([flag, value])
                proc = self._run(*args)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn(expected, proc.stdout)

    def test_generate_workcell_creates_package_folder(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            yaml_path = tmp / "shape.yaml"
            out_dir = tmp / "generated"
            proc = self._run(
                "--template", "sort_by_shape",
                "--cell-name", "Shape",
                "--cell-id", "shape",
                "--robot", "ur5",
                "--end-effector", "robotiq_2f",
                "--camera", "realsense_d435i",
                "--output", str(yaml_path),
                "--generate-workcell",
                "--workcell-output-dir", str(out_dir),
                "--package-name", "generated_shape",
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            package_dir = out_dir / "generated_shape"
            self.assertTrue(package_dir.is_dir())
            manifest, _, _ = scene_contract_validator._read_manifest(str(package_dir / "scene_manifest.yaml"))
            status, _ = scene_contract_validator.validate_task_recipe_block(manifest)
            self.assertIn(status, {"PASS", "WARN"})

    def test_non_interactive_ur5_suction_wizard_generation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            yaml_path = tmp / "ur5_suction.yaml"
            out_dir = tmp / "generated"
            proc = self._run(
                "--template", "sort_by_colour",
                "--cell-name", "UR5 Suction Sorting Demo",
                "--cell-id", "ur5_suction_sorting_demo",
                "--robot", "ur5",
                "--end-effector", "suction",
                "--camera", "realsense_d435i",
                "--grasp-strategy", "suction_top_basic",
                "--output", str(yaml_path),
                "--generate-workcell",
                "--workcell-output-dir", str(out_dir),
                "--package-name", "generated_ur5_suction_sorting_from_wizard",
                "--force",
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            valid = subprocess.run([sys.executable, str(self.validator_script), str(yaml_path)], capture_output=True, text=True, check=False)
            self.assertEqual(valid.returncode, 0, msg=valid.stdout + valid.stderr)

    def test_expected_wizard_fixtures_validate(self) -> None:
        fixtures = [
            "wizard_expected_pick_place.yaml",
            "wizard_expected_sort_by_colour.yaml",
            "wizard_expected_sort_by_shape.yaml",
            "wizard_expected_garbage_sorting.yaml",
        ]
        for fixture_name in fixtures:
            with self.subTest(fixture=fixture_name):
                fixture = FIXTURES / fixture_name
                loaded, parser, notes = validator.load_yaml(fixture)
                summary = validator.validate_cell_definition(loaded, fixture, parser, notes)
                self.assertTrue(summary.ok)


if __name__ == "__main__":
    unittest.main()


class CellDefinitionCapabilityRegistryPathTests(unittest.TestCase):
    def test_cli_uses_default_catalog_path(self) -> None:
        fixture = FIXTURES / "cell_definition_sort_by_colour.yaml"
        proc = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "validate_cell_definition.py"), str(fixture)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)

    def test_cli_accepts_fixture_capabilities_dir(self) -> None:
        fixture = FIXTURES / "cell_definition_sort_by_colour.yaml"
        proc = subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts" / "validate_cell_definition.py"),
                str(fixture),
                "--capabilities-dir",
                str(FIXTURES / "capabilities"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
