#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from scripts.generate_workcell_from_cell_definition import generate_package
from scripts.workcell_discovery import discover_generated_workcell_summaries

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO = REPO_ROOT / 'cell_definitions/demo_ur5_sorting_cell.yaml'


class WorkcellBundleApprovalTests(unittest.TestCase):
    def test_generated_bundle_defaults_to_unapproved(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            self.assertEqual(generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False), 0)
            summary = json.loads((out / 'demo_ur5_sorting_cell/generated/generated_workcell_summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['approval']['status'], 'unapproved')

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
