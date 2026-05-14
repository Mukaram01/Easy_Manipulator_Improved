import subprocess, sys, time, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MERGE=ROOT/'scripts'/'workcell_studio_layout_merge.py'
VAL=ROOT/'scripts'/'validate_workcell_studio_generated_scene.py'

def test_stale_detection_layout_newer_than_merge(tmp_path: Path):
    s=tmp_path/'scene_ok'; (s/'layout').mkdir(parents=True); (s/'config').mkdir(); (s/'preview').mkdir(); (s/'smoke').mkdir(); (s/'acceptance').mkdir(); (s/'launch').mkdir();
    for f in ['package.xml','CMakeLists.txt','environment.yaml','scene_manifest.yaml']:
        (s/f).write_text('objects: []\n' if f.endswith('.yaml') else 'x\n')
    (s/'config'/'task_recipe.yaml').write_text('safety: {fake_hardware_first: true, runtime_execution_enabled: false, motion_command_sent: false}\n')
    (s/'config'/'workcell_builder_task_intent.yaml').write_text('safety: {fake_hardware_first: true, runtime_execution_enabled: false, motion_command_sent: false}\n')
    (s/'preview'/'static_preview.html').write_text('ok')
    (s/'smoke'/'offline_smoke_summary.txt').write_text('ok')
    (s/'workcell_studio_summary.json').write_text('{}')
    (s/'launch'/'demo.launch.py').write_text('use_fake_hardware\n')
    (s/'layout'/'workcell_studio_layout.yaml').write_text('saved_at_utc: 2026-01-01T00:00:00Z\nitems: []\n')
    subprocess.run([sys.executable,str(MERGE),str(s)],check=True)
    time.sleep(1)
    (s/'layout'/'workcell_studio_layout.yaml').write_text('saved_at_utc: 2026-01-01T00:00:01Z\nitems: []\n')
    p=subprocess.run([sys.executable,str(VAL),str(s),'--json'],capture_output=True,text=True,check=False)
    out=json.loads(p.stdout)
    assert any('stale' in w.lower() for w in out['warnings'])
