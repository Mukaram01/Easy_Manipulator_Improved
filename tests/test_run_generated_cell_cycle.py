#!/usr/bin/env python3
from __future__ import annotations
import json, subprocess, sys, tempfile, unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / 'scripts' / 'run_generated_cell_cycle.py'
TASK = REPO_ROOT / 'tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml'
TASK_WARN = REPO_ROOT / 'tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting_missing_reject_pose.yaml'
OBJECTS = REPO_ROOT / 'tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml'

class RunGeneratedCellCycleTests(unittest.TestCase):
    def _run(self, *args: str):
        return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True, check=False)

    def test_fixture_path_generates_outputs(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--json')
            self.assertEqual(p.returncode,0,msg=p.stdout+p.stderr)
            self.assertTrue((Path(td)/'cycle_report.json').exists())
            self.assertTrue((Path(td)/'emd_grasp_bridge_payload.json').exists())

    def test_dry_run_skips_replay(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--replay','--json')
            report=json.loads(p.stdout)
            self.assertEqual(report['replay_status'],'SKIPPED')

    def test_missing_detected_objects_fails(self):
        p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK))
        self.assertNotEqual(p.returncode,0)
        self.assertIn('--detected-objects is required',p.stderr+p.stdout)

    def test_capture_live_with_fake_input(self):
        with tempfile.TemporaryDirectory() as td:
            fake=Path('/tmp/mvp1/fake_detected_objects.yaml')
            fake.parent.mkdir(parents=True,exist_ok=True)
            fake.write_text(OBJECTS.read_text(encoding='utf-8'), encoding='utf-8')
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--capture-live','--output-dir',td,'--dry-run','--json')
            self.assertEqual(p.returncode,0,msg=p.stdout+p.stderr)
            self.assertTrue((Path(td)/'live_detected_objects.yaml').exists())

    def test_strict_escalates_warning(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK_WARN),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--strict','--json')
            self.assertNotEqual(p.returncode,0)

    def test_report_has_status(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--json')
            report=json.loads(p.stdout)
            self.assertIn(report['status'],{'PASS','WARN','FAIL'})

if __name__ == '__main__':
    unittest.main()
