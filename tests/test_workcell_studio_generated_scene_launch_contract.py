import json, subprocess, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/validate_workcell_studio_generated_scene.py'

def test_placeholder_preview_only(tmp_path: Path):
    s=tmp_path/'placeholder_scene'
    (s/'config').mkdir(parents=True); (s/'preview').mkdir()
    for f in ['package.xml','CMakeLists.txt','environment.yaml','scene_manifest.yaml']:
        (s/f).write_text('x')
    (s/'config/task_recipe.yaml').write_text('safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n')
    (s/'config/workcell_builder_task_intent.yaml').write_text('safety:\n  fake_hardware_first: true\n  runtime_execution_enabled: false\n  motion_command_sent: false\n')
    (s/'preview/static_preview.html').write_text('x')
    out=json.loads(subprocess.run([sys.executable,str(SCRIPT),str(s),'--json'],capture_output=True,text=True).stdout)
    assert out['status']=='PREVIEW_ONLY'
