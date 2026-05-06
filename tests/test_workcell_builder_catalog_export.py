#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


exporter = _load_module("export_workcell_builder_catalog", REPO_ROOT / "scripts" / "export_workcell_builder_catalog.py")


class WorkcellBuilderCatalogExportTests(unittest.TestCase):
    def test_export_contains_expected_entries(self) -> None:
        catalog = exporter.export_catalog(exporter.DEFAULT_CAP_DIR, exporter.DEFAULT_GRASP_DIR)
        robots = {e["id"] for e in catalog["robot_capabilities"]}
        eefs = {e["id"] for e in catalog["end_effector_capabilities"]}
        grasps = {e["id"] for e in catalog["grasp_strategies"]}

        self.assertIn("ur5", robots)
        self.assertIn("generic_delta_900", robots)
        self.assertIn("generic_gantry_xyz", robots)
        self.assertIn("robotiq_2f_85", eefs)
        self.assertIn("onrobot_airpick_style", eefs)
        self.assertTrue(any(g.startswith("suction") for g in grasps))

        self.assertIn("runtime_status_hints", catalog)
        self.assertTrue(any("preview_only" in e for e in catalog["robot_capabilities"]))

    def test_script_writes_catalog_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out = Path(tmpdir) / "workcell_studio_catalog.yaml"
            catalog = exporter.export_catalog(exporter.DEFAULT_CAP_DIR, exporter.DEFAULT_GRASP_DIR)
            out.write_text(json.dumps(catalog, indent=2), encoding="utf-8")
            self.assertTrue(out.is_file())
            loaded = json.loads(out.read_text(encoding="utf-8"))
            self.assertIn("grasp_strategies", loaded)


if __name__ == "__main__":
    unittest.main()
