from pathlib import Path
import shutil, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/workcell_studio_demo_mode.py'

def test_dashboard_sections(tmp_path: Path):
    src=next((ROOT/'scenes').iterdir())
    scene=tmp_path/src.name
    shutil.copytree(src, scene)
    subprocess.run([sys.executable,str(SCRIPT),str(scene),'--json'],capture_output=True,text=True,check=False)
    html=(scene/'demo'/'workcell_studio_demo_dashboard.html').read_text(encoding='utf-8')
    for token in ['Workcell Studio Demo Readiness','Scene Overview','Acceptance','Smoke Check','Safety Banner']:
        assert token in html
