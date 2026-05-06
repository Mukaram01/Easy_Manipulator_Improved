from __future__ import annotations
import json, subprocess, sys, tempfile, unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / 'scripts' / 'validate_grasp_strategy.py'
CATALOG = REPO_ROOT / 'catalog' / 'grasp_strategies'

class GraspStrategyTests(unittest.TestCase):
    def run_cmd(self,*args):
        return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True, check=False)

    def test_catalog_files_pass(self):
        p=self.run_cmd(str(CATALOG)); self.assertEqual(p.returncode,0)
    def test_single_file_passes(self):
        p=self.run_cmd(str(CATALOG/'suction_top_basic.yaml')); self.assertEqual(p.returncode,0)
    def test_missing_schema_version_fails(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'bad.yaml'; f.write_text('grasp_strategy: {}\n')
            p=self.run_cmd(str(f)); self.assertNotEqual(p.returncode,0)
    def test_missing_grasp_strategy_fails(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'bad.yaml'; f.write_text('schema_version: grasp_strategy/v1\n')
            p=self.run_cmd(str(f)); self.assertNotEqual(p.returncode,0)
    def test_non_positive_approach_fails(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'bad.yaml'; f.write_text((CATALOG/'suction_top_basic.yaml').read_text().replace('approach_distance_m: 0.12','approach_distance_m: 0.0'))
            p=self.run_cmd(str(f)); self.assertNotEqual(p.returncode,0)
    def test_unknown_ee_warn_default_fail_strict(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'warn.yaml'; t=(CATALOG/'suction_top_basic.yaml').read_text().replace('onrobot_airpick_style','unknown_ee')
            f.write_text(t)
            p=self.run_cmd(str(f)); self.assertEqual(p.returncode,0); self.assertIn('WARN',p.stdout)
            s=self.run_cmd(str(f),'--strict'); self.assertNotEqual(s.returncode,0)
    def test_json_output(self):
        p=self.run_cmd(str(CATALOG),'--json'); self.assertEqual(p.returncode,0); o=json.loads(p.stdout); self.assertIn('summary',o); self.assertIn('results',o)
    def test_recursive_dir(self):
        with tempfile.TemporaryDirectory() as d:
            nested=Path(d)/'a'/'b'; nested.mkdir(parents=True)
            (nested/'x.yaml').write_text((CATALOG/'finger_pinch_basic.yaml').read_text())
            p=self.run_cmd(d); self.assertEqual(p.returncode,0)

if __name__=='__main__': unittest.main()
