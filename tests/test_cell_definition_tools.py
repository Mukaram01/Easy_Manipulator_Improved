#!/usr/bin/env python3
"""Tests for Cell Definition v1 validation and preview tooling."""

from __future__ import annotations

import importlib.util
import sys
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


if __name__ == "__main__":
    unittest.main()
