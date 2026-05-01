#!/usr/bin/env python3
from __future__ import annotations
import json, tempfile, unittest
from pathlib import Path

from scripts.run_scenario_pack import load_and_validate_scenario, run_scenario
from scripts.studio_lite import build_run_scenario_matrix_command, build_run_scenario_pack_command
from scripts.workcell_discovery import discover_scenario_packs

ROOT = Path(__file__).resolve().parents[1]

class ScenarioPackTests(unittest.TestCase):
    def test_valid_scenario_pack_validates(self):
        s, errs = load_and_validate_scenario(ROOT / 'scenario_packs/ur5_2f_garbage_sorting.yaml')
        self.assertEqual(errs, [])
        self.assertEqual(s['schema_version'], 'scenario_pack/v1')

    def test_missing_cell_definition_fails_clearly(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'bad.yaml'
            p.write_text('schema_version: scenario_pack/v1\nname: bad\nenabled: true\ncell_definition: missing.yaml\nexpected:\n  task_type: pick_place\n', encoding='utf-8')
            rep = run_scenario(p, Path(td) / 'out', True)
            self.assertEqual(rep['status'], 'FAIL')
            self.assertTrue(any('cell_definition not found' in b for b in rep['blockers']))

    def test_disabled_scenario_is_skipped(self):
        rep = run_scenario(ROOT / 'scenario_packs/ur5_suction_pick_place.yaml', Path('/tmp/scenario_runs_test'), True)
        self.assertEqual(rep['status'], 'SKIPPED')

    def test_runner_creates_report_and_safe_false(self):
        with tempfile.TemporaryDirectory() as td:
            rep = run_scenario(ROOT / 'scenario_packs/ur5_2f_garbage_sorting.yaml', Path(td), True)
            report = Path(td) / rep['scenario'] / 'scenario_run_report.json'
            report.write_text(json.dumps(rep), encoding='utf-8')
            self.assertTrue(report.exists())
            self.assertFalse(rep['safe_for_robot_motion'])
            self.assertIn(rep['steps']['generate_workcell'], {'PASS', 'WARN'})
            self.assertIn(rep['steps']['visual_preview'], {'PASS', 'WARN'})
            self.assertIn(rep['steps']['gated_dry_run'], {'PASS', 'WARN'})

    def test_live_scenario_pack_validates(self):
        s, errs = load_and_validate_scenario(ROOT / 'scenario_packs/ur5_2f_live_garbage_sorting.yaml')
        self.assertEqual(errs, [])
        self.assertEqual(s['expected']['allowed_destinations'], ['plastic_dest', 'metal_dest', 'reject_dest'])

    def test_command_builders_and_discovery(self):
        self.assertIn('run_scenario_pack.py', ' '.join(build_run_scenario_pack_command('a.yaml', '/tmp/o')))
        self.assertIn('run_scenario_matrix.py', ' '.join(build_run_scenario_matrix_command('scenario_packs', '/tmp/o')))
        records = discover_scenario_packs()
        self.assertTrue(any(r.name == 'ur5_2f_garbage_sorting' for r in records))

if __name__ == '__main__':
    unittest.main()
