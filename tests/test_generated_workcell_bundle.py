#!/usr/bin/env python3
from __future__ import annotations
import json, tempfile, unittest
from pathlib import Path

from scripts.generate_workcell_from_cell_definition import generate_package
from scripts.run_generated_workcell_bundle import build_command
from scripts.run_cell_cycle_panel import build_run_generated_workcell_bundle_command
from scripts.validate_detected_objects import _load_yaml_or_json
from scripts.workcell_discovery import discover_generated_workcell_summaries

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO = REPO_ROOT / 'cell_definitions/demo_ur5_sorting_cell.yaml'


class GeneratedWorkcellBundleTests(unittest.TestCase):
    def test_bundle_files_created_and_detected_valid(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            rc = generate_package(DEMO, out, 'demo_ur5_sorting_cell', force=True, dry_run=False)
            self.assertEqual(rc, 0)
            pkg = out / 'demo_ur5_sorting_cell' / 'generated'
            for name in ['generated_workcell_summary.json','generated_detected_objects_example.yaml','generated_environment_objects.yaml','generated_destinations.yaml','generated_gated_dry_run_command.sh']:
                self.assertTrue((pkg / name).exists(), name)
            data, _, _ = _load_yaml_or_json(pkg / 'generated_detected_objects_example.yaml')
            self.assertEqual(data.get('schema_version'), 'detected_objects/v1')
            cmd_txt = (pkg / 'generated_gated_dry_run_command.sh').read_text(encoding='utf-8')
            self.assertIn('--require-preflight', cmd_txt)
            self.assertIn('--dry-run', cmd_txt)
            self.assertIn('--no-replay', cmd_txt)
            summary = json.loads((pkg / 'generated_workcell_summary.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['approval']['status'], 'unapproved')

    def test_run_bundle_build_command(self):
        summary = {
            'scene_package': 'demo',
            'runtime_scene_package': 'ur5_2f_test',
            'task_recipe_path': '/tmp/a/task.yaml',
            'detected_objects_example_path': '/tmp/a/detected.yaml',
        }
        cmd = build_command(summary, Path('/tmp/out'), True, True, True, True, False, {})
        self.assertIn('--require-preflight', cmd)
        self.assertIn('--dry-run', cmd)
        self.assertIn('--no-replay', cmd)
        self.assertIn('ur5_2f_test', cmd)

    def test_run_bundle_build_command_live_mode(self):
        summary = {'scene_package': 'demo', 'runtime_scene_package': 'ur5_2f_test', 'task_recipe_path': '/tmp/a/task.yaml'}
        live_args = {
            'epd_topic': '/easy_perception_deployment/epd_localize_output',
            'epd_qos_reliability': 'best_effort',
            'epd_qos_depth': 10,
            'capture_timeout': 10.0,
            'target_frame': 'world',
            'tf_timeout': 2.0,
            'allow_untransformed': False,
            'preflight_live': True,
            'preflight_check_tf': True,
            'preflight_check_ros_topics': True,
            'preflight_camera_frame': 'camera_depth_optical_frame',
        }
        cmd = build_command(summary, Path('/tmp/out'), True, True, True, True, True, live_args)
        for token in ['--capture-live', '--epd-topic', '--epd-qos-reliability', 'best_effort', '--target-frame', 'world', '--require-transform', '--preflight-live', '--preflight-check-tf', '--preflight-check-ros-topics']:
            self.assertIn(token, cmd)

    def test_missing_summary_fails_clearly(self):
        script = REPO_ROOT / 'scripts/run_generated_workcell_bundle.py'
        with tempfile.TemporaryDirectory() as td:
            import subprocess, sys
            p = subprocess.run([sys.executable, str(script), '--workcell', td, '--gated-dry-run', '--json'], capture_output=True, text=True, check=False)
            self.assertNotEqual(p.returncode, 0)
            self.assertIn('missing generated summary', p.stderr + p.stdout)

    def test_panel_bundle_command_builder(self):
        cmd = build_run_generated_workcell_bundle_command('/tmp/generated/demo')
        self.assertIn('run_generated_workcell_bundle.py', ' '.join(cmd))

    def test_discovery_generated_bundle_shape(self):
        summaries = discover_generated_workcell_summaries()
        self.assertIsInstance(summaries, list)

if __name__ == '__main__':
    unittest.main()
