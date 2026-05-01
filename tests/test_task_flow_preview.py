#!/usr/bin/env python3
from __future__ import annotations
import json, subprocess, sys, tempfile, unittest
from pathlib import Path

from scripts.preview_generated_workcell_bundle import build_marker_specs
from scripts.run_cell_cycle_panel import build_preview_task_flow_command
from scripts.run_generated_workcell_bundle import build_command

REPO_ROOT = Path(__file__).resolve().parents[1]
CYCLE = REPO_ROOT / 'scripts/run_generated_cell_cycle.py'
TASK = REPO_ROOT / 'tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml'
TASK_MISS = REPO_ROOT / 'tests/fixtures/task_recipes/fail_missing_destination.yaml'
OBJECTS = REPO_ROOT / 'tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml'

class TaskFlowPreviewTests(unittest.TestCase):
    def test_cycle_writes_task_flow_preview(self):
        with tempfile.TemporaryDirectory() as td:
            p = subprocess.run([sys.executable, str(CYCLE), '--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--json'], capture_output=True, text=True, check=False)
            self.assertEqual(p.returncode, 0, msg=p.stdout + p.stderr)
            tf = Path(td) / 'task_flow_preview.json'
            self.assertTrue(tf.exists())
            data = json.loads(tf.read_text())
            self.assertEqual(data['schema_version'], 'task_flow_preview/v1')
            self.assertFalse(data['safe_for_robot_motion'])
            self.assertTrue(data['selected_object'])
            self.assertTrue(any(s['type'] == 'pick' for s in data['steps']))
            self.assertTrue(any(s['type'] == 'release' for s in data['steps']))

    def test_missing_destination_warns_not_crash(self):
        with tempfile.TemporaryDirectory() as td:
            p = subprocess.run([sys.executable, str(CYCLE), '--scene-package','ur5_2f_test','--task-recipe',str(TASK_MISS),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--json'], capture_output=True, text=True, check=False)
            tf = json.loads((Path(td) / 'task_flow_preview.json').read_text())
            self.assertTrue(tf['warnings'] or tf['blockers'])

    def test_preview_loads_task_flow_and_generates_markers_json_only(self):
        summary = {'planning_frame': 'world'}
        env = {'objects': []}
        dest = {'destinations': []}
        det = {'objects': []}
        flow = {'steps': [{'index': 1, 'name': 'pick', 'xyz': [0,0,0]}, {'index': 2, 'name': 'release', 'xyz': [1,0,0]}], 'selected_object': 'a', 'selected_destination': 'b'}
        markers, task = build_marker_specs(summary, env, dest, det, 'a', 'b', task_flow=flow, show_task_flow=True)
        self.assertGreater(len([m for m in markers if m['ns'].startswith('task_flow')]), 0)
        self.assertEqual(task['task_flow_steps'], 2)

    def test_bundle_command_passes_write_task_flow(self):
        cmd = build_command({'scene_package':'s','task_recipe_path':'t','detected_objects_example_path':'d'}, Path('/tmp/o'), True, True, True, True)
        self.assertIn('--write-task-flow-preview', cmd)

    def test_panel_builder_has_flags(self):
        cmd = build_preview_task_flow_command('/tmp/w', '/tmp/out')
        self.assertIn('--show-task-flow', cmd)
        self.assertIn('--task-flow-preview', cmd)

if __name__ == '__main__':
    unittest.main()
