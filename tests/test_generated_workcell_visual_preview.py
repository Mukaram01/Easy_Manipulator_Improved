#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from scripts.generate_workcell_from_cell_definition import generate_package
from scripts.preview_generated_workcell_bundle import build_marker_specs
from scripts.run_cell_cycle_panel import build_preview_generated_workcell_bundle_command
from scripts.run_generated_workcell_bundle import build_preview_command

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO = REPO_ROOT / 'cell_definitions/demo_ur5_sorting_cell.yaml'


class GeneratedWorkcellVisualPreviewTests(unittest.TestCase):
    def test_missing_summary_fails_clearly(self):
        script = REPO_ROOT / 'scripts/preview_generated_workcell_bundle.py'
        with tempfile.TemporaryDirectory() as td:
            p = subprocess.run([sys.executable, str(script), '--workcell', td, '--json'], capture_output=True, text=True, check=False)
            self.assertNotEqual(p.returncode, 0)
            self.assertIn('missing required file', p.stderr + p.stdout)

    def test_valid_bundle_generates_visual_preview_summary(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            self.assertEqual(generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False), 0)
            pkg = out / 'demo_ur5_sorting_cell'
            script = REPO_ROOT / 'scripts/preview_generated_workcell_bundle.py'
            p = subprocess.run([sys.executable, str(script), '--workcell', str(pkg), '--json'], capture_output=True, text=True, check=False)
            self.assertEqual(p.returncode, 0)
            summary = json.loads((pkg / 'generated/visual_preview_summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['schema_version'], 'generated_workcell_visual_preview/v1')
            self.assertGreaterEqual(summary['objects_visualized'], 1)
            self.assertGreaterEqual(summary['destinations_visualized'], 1)

    def test_task_preview_and_builders(self):
        summary = {'planning_frame': 'world'}
        env = {'objects': [{'id': 'table', 'dimensions': {'x': 1, 'y': 1, 'z': 0.1}, 'pose': {'position': {'x': 0, 'y': 0, 'z': 0}}}]}
        dest = {'destinations': [{'id': 'plastic_bin', 'pose': {'position': {'x': 1, 'y': 0, 'z': 0.5}}}]}
        det = {'objects': [{'object_id': 'rec_001', 'pose': {'position': {'x': 0.2, 'y': 0.1, 'z': 0.4}}}]}
        markers, task = build_marker_specs(summary, env, dest, det, 'rec_001', 'plastic_bin')
        self.assertTrue(any(m['ns'] == 'environment' for m in markers))
        self.assertTrue(any(m['ns'] == 'destination' for m in markers))
        self.assertTrue(any(m['ns'] == 'detected' for m in markers))
        self.assertEqual(task['selected_object'], 'rec_001')
        self.assertEqual(task['selected_destination'], 'plastic_bin')

        cmd = build_preview_command(Path('/tmp/generated/demo'), False, True)
        self.assertIn('preview_generated_workcell_bundle.py', ' '.join(cmd))
        panel_cmd = build_preview_generated_workcell_bundle_command('/tmp/generated/demo')
        self.assertIn('preview_generated_workcell_bundle.py', ' '.join(panel_cmd))


if __name__ == '__main__':
    unittest.main()
