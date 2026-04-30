#!/usr/bin/env python3
from __future__ import annotations
import json, subprocess, sys, tempfile, unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / 'scripts' / 'run_generated_cell_cycle.py'
TASK = REPO_ROOT / 'tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting.yaml'
TASK_WARN = REPO_ROOT / 'tests/fixtures/task_recipes/mvp1_generated_cell_colour_sorting_missing_reject_pose.yaml'
OBJECTS = REPO_ROOT / 'tests/fixtures/detected_objects/mvp1_colour_sorting_with_fallback.yaml'
TASK_VALID_GARBAGE = REPO_ROOT / 'tests/fixtures/task_recipes/valid_garbage_sorting.yaml'
OBJECTS_VALID_GARBAGE = REPO_ROOT / 'tests/fixtures/detected_objects/valid_epd_garbage_sorting.yaml'

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
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--capture-live','--offline-fake-live','--output-dir',td,'--dry-run','--json')
            self.assertEqual(p.returncode,0,msg=p.stdout+p.stderr)
            self.assertTrue((Path(td)/'live_detected_objects.yaml').exists())

    def test_capture_live_without_explicit_fake_mode_fails(self):
        with tempfile.TemporaryDirectory() as td:
            fake=Path('/tmp/mvp1/fake_detected_objects.yaml')
            fake.parent.mkdir(parents=True,exist_ok=True)
            fake.write_text(OBJECTS.read_text(encoding='utf-8'), encoding='utf-8')
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--capture-live','--output-dir',td,'--dry-run','--json')
            self.assertNotEqual(p.returncode,0)
            self.assertIn('live EPD capture failed', p.stderr + p.stdout)

    def test_strict_escalates_warning(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK_WARN),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--strict','--json')
            self.assertNotEqual(p.returncode,0)

    def test_report_has_status(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK),'--detected-objects',str(OBJECTS),'--output-dir',td,'--dry-run','--json')
            report=json.loads(p.stdout)
            self.assertIn(report['status'],{'PASS','WARN','FAIL'})
            self.assertIn('object_pose_frame_raw', report)
            self.assertIn('object_pose_frame_normalized', report)
            self.assertIn('transform_status', report)

    def test_valid_garbage_offline_cycle_has_no_blockers(self):
        with tempfile.TemporaryDirectory() as td:
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK_VALID_GARBAGE),'--detected-objects',str(OBJECTS_VALID_GARBAGE),'--output-dir',td,'--dry-run','--no-replay','--json')
            self.assertEqual(p.returncode,0,msg=p.stdout+p.stderr)
            report=json.loads(p.stdout)
            self.assertIn(report['status'],{'PASS','WARN'})
            self.assertEqual(report['acceptance'].get('blockers',[]),[])


    def test_report_includes_live_fields_and_snapshot_path(self):
        with tempfile.TemporaryDirectory() as td:
            fake=Path('/tmp/mvp1/fake_detected_objects.yaml')
            fake.parent.mkdir(parents=True,exist_ok=True)
            fake.write_text(OBJECTS_VALID_GARBAGE.read_text(encoding='utf-8'), encoding='utf-8')
            p=self._run('--scene-package','ur5_2f_test','--task-recipe',str(TASK_VALID_GARBAGE),'--capture-live','--offline-fake-live','--epd-topic','/easy_perception_deployment/epd_localize_output','--output-dir',td,'--dry-run','--no-replay','--json')
            self.assertEqual(p.returncode,0,msg=p.stdout+p.stderr)
            report=json.loads(p.stdout)
            self.assertNotEqual(report['perception_source'],'live_epd')
            self.assertIn(report['capture_status'], {'offline_fake_file', 'offline_fake_message'})
            self.assertTrue(report['detected_objects_used'].endswith('detected_objects_used.yaml'))

if __name__ == '__main__':
    unittest.main()
