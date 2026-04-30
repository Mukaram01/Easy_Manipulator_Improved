#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURES = REPO_ROOT / "tests" / "fixtures"
TASK_FIXTURES = FIXTURES / "task_recipes"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


task_validator = _load("validate_task_recipe_mod", REPO_ROOT / "scripts" / "validate_task_recipe.py")
cell_validator = _load("validate_cell_definition_with_recipe", REPO_ROOT / "scripts" / "validate_cell_definition.py")
generator = REPO_ROOT / "scripts" / "generate_task_recipe_from_cell_definition.py"


class TaskRecipeValidatorTests(unittest.TestCase):
    def test_valid_fixtures_pass(self) -> None:
        for name in [
            "valid_pick_place.yaml",
            "valid_sort_by_colour.yaml",
            "valid_sort_by_shape.yaml",
            "valid_garbage_sorting.yaml",
            "valid_inspection_then_place.yaml",
        ]:
            summary = task_validator.validate_path(TASK_FIXTURES / name)
            self.assertTrue(summary.ok, msg=name)

    def test_fail_fixtures_fail(self) -> None:
        for name in ["fail_missing_destination.yaml", "fail_bad_schema.yaml", "fail_rule_unknown_destination.yaml"]:
            summary = task_validator.validate_path(TASK_FIXTURES / name)
            self.assertFalse(summary.ok, msg=name)

    def test_warning_fixtures_warn(self) -> None:
        for name in ["warn_missing_fallback.yaml", "warn_missing_reject_destination.yaml"]:
            summary = task_validator.validate_path(TASK_FIXTURES / name)
            self.assertTrue(summary.ok, msg=name)
            self.assertTrue(summary.warnings)

    def test_strict_converts_warning_to_fail(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "validate_task_recipe.py"), str(TASK_FIXTURES / "warn_missing_fallback.yaml"), "--strict"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(proc.returncode, 0)

    def test_json_output_is_valid(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "validate_task_recipe.py"), str(TASK_FIXTURES / "valid_pick_place.yaml"), "--json"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
        payload = json.loads(proc.stdout)
        self.assertIn("result", payload)
        self.assertIn("files", payload)

    def test_directory_validation_works(self) -> None:
        proc = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "validate_task_recipe.py"), str(TASK_FIXTURES), "--json"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(proc.returncode, 0)
        payload = json.loads(proc.stdout)
        self.assertEqual(len(payload["files"]), 12)


class TaskRecipeGeneratorTests(unittest.TestCase):
    def test_generator_from_cell_definition_works(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "task_recipe.preview.yaml"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(generator),
                    str(FIXTURES / "cell_definition_sort_by_colour.yaml"),
                    "--output",
                    str(out),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(proc.returncode, 0, msg=proc.stdout + proc.stderr)
            self.assertTrue(out.is_file())
            summary = task_validator.validate_path(out)
            self.assertTrue(summary.ok)


class CellDefinitionRecipeIntegrationTests(unittest.TestCase):
    def test_backward_compatibility_without_task_recipe(self) -> None:
        loaded, parser, notes = cell_validator.load_yaml(FIXTURES / "cell_definition_pick_place.yaml")
        summary = cell_validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_pick_place.yaml", parser, notes)
        self.assertTrue(summary.ok)

    def test_cell_definition_with_embedded_task_recipe_validates(self) -> None:
        loaded, parser, notes = cell_validator.load_yaml(FIXTURES / "cell_definition_pick_place.yaml")
        embedded, _, _ = cell_validator.load_yaml(TASK_FIXTURES / "valid_pick_place.yaml")
        loaded["task"]["recipe"] = embedded
        summary = cell_validator.validate_cell_definition(loaded, FIXTURES / "cell_definition_pick_place.yaml", parser, notes)
        self.assertTrue(summary.ok, msg="\n".join(summary.errors + summary.warnings))

    def test_cell_definition_with_external_task_recipe_validates(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            cell_path = tmp / "cell.yaml"
            recipe_path = tmp / "recipe.yaml"
            cell_path.write_text((FIXTURES / "cell_definition_sort_by_colour.yaml").read_text(encoding="utf-8"), encoding="utf-8")
            recipe_path.write_text((TASK_FIXTURES / "valid_sort_by_colour.yaml").read_text(encoding="utf-8"), encoding="utf-8")
            loaded, parser, notes = cell_validator.load_yaml(cell_path)
            loaded["task"]["recipe"] = "recipe.yaml"
            summary = cell_validator.validate_cell_definition(loaded, cell_path, parser, notes)
            self.assertTrue(summary.ok, msg="\n".join(summary.errors + summary.warnings))

    def test_unknown_destination_in_decision_rule_fails(self) -> None:
        summary = task_validator.validate_path(TASK_FIXTURES / "fail_rule_unknown_destination.yaml")
        self.assertFalse(summary.ok)
        self.assertIn("ghost_bin", " ".join(summary.errors))


if __name__ == "__main__":
    unittest.main()
