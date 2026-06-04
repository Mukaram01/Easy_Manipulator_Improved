#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

from scripts.generate_workcell_from_cell_definition import generate_package
from scripts.workcell_discovery import discover_generated_workcell_summaries

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO = REPO_ROOT / 'cell_definitions/demo_ur5_sorting_cell.yaml'
FIXTURE_STL = REPO_ROOT / 'tests/fixtures/meshes/tiny_ascii_cube.stl'


class WorkcellBundleApprovalTests(unittest.TestCase):
    def test_generated_bundle_defaults_to_unapproved(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            self.assertEqual(generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False), 0)
            summary = json.loads((out / 'demo_ur5_sorting_cell/generated/generated_workcell_summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['approval']['status'], 'unapproved')


    def test_generated_bundle_includes_preview_safe_visual_mesh_index_fallback(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            self.assertEqual(generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False), 0)
            workcell = out / 'demo_ur5_sorting_cell'
            index_path = workcell / 'generated/scene_visual_mesh_index.json'
            self.assertTrue(index_path.exists())
            index = json.loads(index_path.read_text(encoding='utf-8'))
            self.assertEqual(index['schema_version'], 'scene_visual_mesh_index/v1')
            self.assertTrue(index['safe_for_preview'])
            self.assertEqual(index['visual_items'], [])
            self.assertTrue(any('no meshes were found' in warning for warning in index.get('warnings', [])))
            report = (workcell / 'generated/validation_report.md').read_text(encoding='utf-8')
            self.assertIn('Visual mesh extraction fallback', report)
            self.assertTrue((workcell / 'urdf/scene.urdf.xacro').exists())


    def test_generated_bundle_uses_extracted_visual_mesh_index_when_meshes_exist(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            cell_definition = yaml.safe_load(DEMO.read_text(encoding='utf-8'))
            cell_definition['objects'][0]['mesh'] = str(FIXTURE_STL)
            cell_path = out / 'mesh_cell.yaml'
            cell_path.write_text(yaml.safe_dump(cell_definition, sort_keys=False), encoding='utf-8')
            self.assertEqual(generate_package(cell_path, out, 'mesh_cell', force=True, dry_run=False), 0)
            index = json.loads((out / 'mesh_cell/generated/scene_visual_mesh_index.json').read_text(encoding='utf-8'))
            self.assertEqual(index['schema_version'], 'scene_visual_mesh_index/v1')
            self.assertGreaterEqual(index['candidate_mesh_count'], 1)
            self.assertTrue(index['visual_items'])
            self.assertEqual(index['visual_items'][0]['geometry_type'], 'mesh')

    def test_mark_approved_and_report_created(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            self.assertEqual(generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False), 0)
            script = REPO_ROOT / 'scripts/mark_workcell_bundle_approved.py'
            workcell = out / 'demo_ur5_sorting_cell'
            p = subprocess.run([sys.executable, str(script), '--workcell', str(workcell), '--approved-by', 'Integrator', '--notes', 'Offline dry-run reviewed'], capture_output=True, text=True)
            self.assertEqual(p.returncode, 0)
            summary = json.loads((workcell / 'generated/generated_workcell_summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['approval']['status'], 'approved')
            self.assertTrue((workcell / 'generated/approval_report.json').exists())

    def test_discovery_shows_approval_status(self):
        items = discover_generated_workcell_summaries()
        for item in items:
            self.assertIn('approval_status', item)


if __name__ == '__main__':
    unittest.main()
