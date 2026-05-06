#!/usr/bin/env python3
"""Tests for Cell Definition v1 validation and preview/tooling workflows."""

from __future__ import annotations

import importlib.util
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

